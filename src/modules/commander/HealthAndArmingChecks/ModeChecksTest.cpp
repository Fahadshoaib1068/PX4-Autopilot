#include <gtest/gtest.h>

#include "checks/modeCheck.hpp"

class ModeChecksTest : public ::testing::Test
{
protected:
	ModeChecksTest()
		: context(status), reporter(flags, 0_s)
	{
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	}

	vehicle_status_s status{};
	failsafe_flags_s flags{};
	Context context;
	Report reporter;
	ModeChecks checks;
};

TEST_F(ModeChecksTest, angular_velocity_invalid_blocks_required_modes)
{
	flags.angular_velocity_invalid = true;
	flags.mode_req_angular_velocity = static_cast<uint32_t>(NavModes::Stabilized);

	checks.checkAndReport(context, reporter);

	const auto &results = reporter.armingCheckResults();
	EXPECT_EQ(static_cast<uint32_t>(results.can_run & NavModes::Stabilized), 0u);
	EXPECT_NE(static_cast<uint32_t>(results.can_run & NavModes::Manual), 0u);
	EXPECT_EQ(results.error, health_component_t::system);
}

TEST_F(ModeChecksTest, angular_velocity_valid_leaves_modes_runnable)
{
	flags.angular_velocity_invalid = false;
	flags.mode_req_angular_velocity = static_cast<uint32_t>(NavModes::Stabilized);

	checks.checkAndReport(context, reporter);

	const auto &results = reporter.armingCheckResults();
	EXPECT_NE(static_cast<uint32_t>(results.can_run & NavModes::Stabilized), 0u);
	EXPECT_EQ(static_cast<uint32_t>(results.error), 0u);
}