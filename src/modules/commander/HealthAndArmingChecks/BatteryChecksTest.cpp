#include <gtest/gtest.h>

#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/rtl_time_estimate.h>
#include "HealthAndArmingChecks.hpp"

#include "checks/batteryCheck.hpp"

using namespace time_literals;

namespace
{

struct BatteryConfig {
	bool connected{true};
	bool required{false};
	uint8_t warning{battery_status_s::WARNING_NONE};
	float remaining{0.8f};
	float time_remaining_s{NAN};
    uint16_t faults{0};
    hrt_abstime timestamp{0};
};

} // namespace

class BatteryChecksTest : public ::testing::Test
{
protected:
	BatteryChecksTest()
		: battery0(ORB_ID(battery_status)),
		  battery1(ORB_ID(battery_status)),
		  context(status),
		  reporter(flags, 0_s)
	{}

	void SetUp() override
	{
         param_control_autosave(false);
		setParam("COM_ARM_BAT_MIN", 0.15f);
		setParam("CBRK_SUPPLY_CHK", int32_t{0});
		checks.updateParams();

		BatteryConfig idle;
		idle.connected = false;
		publishBattery(0, idle);
		publishBattery(1, idle);
	}

	template<typename T>
	void setParam(const char *name, T value)
	{
		param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		ASSERT_EQ(param_set(handle, &value), 0);
	}

	void publishBattery(int instance, const BatteryConfig &config)
	{
		battery_status_s battery{};
        battery.timestamp = config.timestamp == 0 ? hrt_absolute_time() : config.timestamp;		battery.connected = config.connected;
		battery.is_required = config.required;
		battery.warning = config.warning;
		battery.remaining = config.remaining;
		battery.time_remaining_s = config.time_remaining_s;
		battery.faults = config.faults;

		if (instance == 0) {
			battery0.publish(battery);
		} else {
			battery1.publish(battery);
		}
	}

    void publishRtlEstimate(float safe_time_estimate, bool valid = true)
    {
    rtl_time_estimate_s estimate{};
    estimate.timestamp = hrt_absolute_time();
    estimate.safe_time_estimate = safe_time_estimate;
    estimate.valid = valid;

    rtlTimeEstimate.publish(estimate);
    }

	void run(bool armed)
	{
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		checks.checkAndReport(context, reporter);
	}

	bool hasBatteryHealthError() const
	{
		return ((uint64_t)reporter.healthResults().error & (uint64_t)health_component_t::battery) != 0;
	}

	bool hasBatteryPresent() const
	{
		return ((uint64_t)reporter.healthResults().is_present & (uint64_t)health_component_t::battery) != 0;
	}

	uint32_t canArm() const { return (uint32_t)reporter.armingCheckResults().can_arm; }

	// Instance-bound publications instead of static function variables
	uORB::PublicationMulti<battery_status_s> battery0;
	uORB::PublicationMulti<battery_status_s> battery1;
    uORB::Publication<rtl_time_estimate_s> rtlTimeEstimate{ORB_ID(rtl_time_estimate)};

	vehicle_status_s status{};
	failsafe_flags_s flags{};
	Context context;
	Report reporter;
	BatteryChecks checks;
};

TEST_F(BatteryChecksTest, RequiredBatteryDisconnected_ReportsFailureAndUnhealthy)
{
	BatteryConfig battery;
	battery.connected = false;
	battery.required = true;
	publishBattery(0, battery);

	run(false);

	EXPECT_TRUE(hasBatteryHealthError());
	EXPECT_TRUE(flags.battery_unhealthy);
	EXPECT_EQ(canArm(), 0u);
}

TEST_F(BatteryChecksTest, OptionalBatteryDisconnected_NoFailure)
{
	BatteryConfig battery;
	battery.connected = false;
	battery.required = false;
	publishBattery(0, battery);

	run(false);

	EXPECT_FALSE(hasBatteryHealthError());
	EXPECT_FALSE(flags.battery_unhealthy);
	EXPECT_FALSE(hasBatteryPresent());
	EXPECT_EQ(canArm(), 0xffffffffu);
}

TEST_F(BatteryChecksTest, RequiredBatteryConnected_NoFailure)
{
    BatteryConfig battery;
    battery.connected = true;
    battery.required = true;

    publishBattery(0, battery);
    publishBattery(1, battery);

    run(false);

    EXPECT_FALSE(hasBatteryHealthError());
    EXPECT_FALSE(flags.battery_unhealthy);
    EXPECT_TRUE(hasBatteryPresent());
}

TEST_F(BatteryChecksTest, BatteryFault_ReportsFailureAndUnhealthy)
{
    BatteryConfig battery;
    battery.connected = true;
    battery.required = true;
    battery.faults = 1;

    publishBattery(0, battery);
    publishBattery(1, battery);

    run(false);

    EXPECT_TRUE(hasBatteryHealthError());
    EXPECT_TRUE(flags.battery_unhealthy);
    EXPECT_EQ(canArm(), 0u);
}

TEST_F(BatteryChecksTest, BatteryDisconnectsAfterArming_ReportsFailureAndUnhealthy)
{
        BatteryConfig connected;
        connected.connected = true;
        connected.required = true;

        publishBattery(0, connected);
        publishBattery(1, connected);

        // First run: vehicle arms while the battery is connected.
        run(true);

        BatteryConfig disconnected;
        disconnected.connected = false;
        disconnected.required = true;

        publishBattery(0, disconnected);
        publishBattery(1, disconnected);

        // Second run: battery disconnects while the vehicle is still armed.
        run(true);

        EXPECT_TRUE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0u);
}

TEST_F(BatteryChecksTest, LowBatteryWarning_ReportsFailure)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;
        battery.warning = battery_status_s::WARNING_LOW;
        battery.remaining = 0.5f;

        publishBattery(0, battery);
        publishBattery(1, battery);

        run(false);

        EXPECT_FALSE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0xFFFFFFFFu);
}

TEST_F(BatteryChecksTest, EmergencyBatteryWarning_BlocksArming)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;
        battery.warning = battery_status_s::WARNING_EMERGENCY;
        battery.remaining = 0.10f;

        publishBattery(0, battery);
        publishBattery(1, battery);

        run(false);

        EXPECT_FALSE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0u);
}

TEST_F(BatteryChecksTest, FiniteBatteryTimeRemaining_IsTracked)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;
        battery.time_remaining_s = 120.0f;

        publishBattery(0, battery);
        publishBattery(1, battery);

        run(false);

        EXPECT_FALSE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0xFFFFFFFFu);
}

TEST_F(BatteryChecksTest, LowRemainingFlightTime_BlocksArming)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;
        battery.time_remaining_s = 100.0f;

        publishBattery(0, battery);
        publishBattery(1, battery);

        publishRtlEstimate(120.0f);

        run(true);

        EXPECT_FALSE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0u);
}

TEST_F(BatteryChecksTest, SupplyCheckCircuitBreaker_DisablesBatteryCheck)
{
        setParam("CBRK_SUPPLY_CHK", int32_t{894281});

        run(false);

        EXPECT_FALSE(hasBatteryHealthError());
        EXPECT_FALSE(flags.battery_unhealthy);
        EXPECT_FALSE(hasBatteryPresent());
        EXPECT_EQ(canArm(), 0xFFFFFFFFu);
}

TEST_F(BatteryChecksTest, StaleBatteryUpdate_ReportsUnhealthy)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;

        // Make the battery update older than the 5-second freshness limit.
        battery.timestamp = hrt_absolute_time() - 6_s;

        publishBattery(0, battery);
        publishBattery(1, battery);

        run(false);

        EXPECT_TRUE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0u);
}
TEST_F(BatteryChecksTest, FailedBatteryWarning_ReportsUnhealthy)
{
        BatteryConfig battery;
        battery.connected = true;
        battery.required = true;
        battery.warning = battery_status_s::WARNING_FAILED;

        publishBattery(0, battery);
        publishBattery(1, battery);

        run(false);

        EXPECT_TRUE(hasBatteryHealthError());
        EXPECT_EQ(canArm(), 0u);
}
