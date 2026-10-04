#include <gtest/gtest.h>

#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_status.h>

#include "HealthAndArmingChecks.hpp"
#include "checks/powerCheck.hpp"

using namespace time_literals;

class PowerChecksTest : public ::testing::Test
{
protected:
	PowerChecksTest()
		: systemPower(ORB_ID(system_power)),
		  context(status),
		  reporter(flags, 0_s)
	{}

	void SetUp() override
	{
		param_control_autosave(false);

		setParam("CBRK_SUPPLY_CHK", int32_t{0});
		setParam("COM_POWER_COUNT", int32_t{1});

		checks.updateParams();

		status = {};
		status.power_input_valid = true;
		status.hil_state = vehicle_status_s::HIL_STATE_OFF;
	}

	template<typename T>
	void setParam(const char *name, T value)
	{
		param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		ASSERT_EQ(param_set(handle, &value), 0);
	}

	void publishSystemPower(float voltage = 5.0f,
				uint8_t brick_valid = 1,
				bool usb_connected = false,
				bool hipower_oc = false,
				bool periph_oc = false)
	{
		system_power_s power{};
		power.timestamp = hrt_absolute_time();
		power.voltage5v_v = voltage;
		power.brick_valid = brick_valid;
		power.usb_connected = usb_connected;
		power.hipower_5v_oc = hipower_oc;
		power.periph_5v_oc = periph_oc;

		systemPower.publish(power);
	}

	void run(bool armed = false)
	{
		status.arming_state = armed
					   ? vehicle_status_s::ARMING_STATE_ARMED
					   : vehicle_status_s::ARMING_STATE_DISARMED;

		checks.checkAndReport(context, reporter);
	}

	bool hasHealthError() const
	{
		return reporter.healthResults().error != health_component_t{};
	}

	uint32_t canArm() const
	{
		return (uint32_t)reporter.armingCheckResults().can_arm;
	}

	uORB::Publication<system_power_s> systemPower;

	vehicle_status_s status{};
	failsafe_flags_s flags{};
	Context context;
	Report reporter;
	PowerChecks checks;
};

TEST_F(PowerChecksTest, InvalidPowerInput_ReportsHealthFailure)
{
	status.power_input_valid = false;

	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, MissingSystemPower_ReportsHealthFailure)
{
	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, LowVoltage_ReportsHealthFailure)
{
	publishSystemPower(4.5f);

	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, HighVoltage_ReportsHealthFailure)
{
	publishSystemPower(5.6f);

	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, InsufficientPowerModules_BlocksArming)
{
	setParam("COM_POWER_COUNT", int32_t{2});
	checks.updateParams();

	publishSystemPower(5.0f, 1);

	run();

	EXPECT_EQ(canArm(), 0u);
}

TEST_F(PowerChecksTest, HighPowerOvercurrent_ReportsHealthFailure)
{
	publishSystemPower(5.0f, 1, false, true, false);

	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, PeripheralOvercurrent_ReportsHealthFailure)
{
	publishSystemPower(5.0f, 1, false, false, true);

	run();

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, OvercurrentWhileArmed_ReportsHealthFailure)
{
	publishSystemPower(5.0f, 1, false, true, false);

	run(true);

	EXPECT_TRUE(hasHealthError());
}

TEST_F(PowerChecksTest, HilMode_BypassesPowerCheck)
{
	status.hil_state = vehicle_status_s::HIL_STATE_ON;

	run();

	EXPECT_FALSE(hasHealthError());
}

TEST_F(PowerChecksTest, CircuitBreaker_BypassesPowerCheck)
{
	setParam("CBRK_SUPPLY_CHK", int32_t{894281});
	checks.updateParams();

	run();

	EXPECT_FALSE(hasHealthError());
}
