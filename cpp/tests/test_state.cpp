#include <gtest/gtest.h>
#include <objects/State.hpp>

// Sanity check: confirms the test binary itself runs under CI.
TEST(Sanity, BasicMath) {
    EXPECT_EQ(1 + 1, 2);
}

TEST(State, DefaultsToZeroAndIdentity) {
    State s;
    EXPECT_EQ(s.pos_i, Eigen::Vector3d::Zero());
    EXPECT_EQ(s.vel_b, Eigen::Vector3d::Zero());
    EXPECT_EQ(s.omega_b, Eigen::Vector3d::Zero());
    EXPECT_TRUE(s.q_bi.isApprox(Eigen::Quaterniond::Identity()));
}

TEST(State, ScalarMultiplyScalesVectors) {
    State s;
    s.pos_i = Eigen::Vector3d(1.0, 2.0, 3.0);
    s.vel_b = Eigen::Vector3d(4.0, 5.0, 6.0);

    State scaled = s * 2.0;

    EXPECT_TRUE(scaled.pos_i.isApprox(Eigen::Vector3d(2.0, 4.0, 6.0)));
    EXPECT_TRUE(scaled.vel_b.isApprox(Eigen::Vector3d(8.0, 10.0, 12.0)));
}

TEST(State, AdditionSumsVectors) {
    State a;
    a.pos_i = Eigen::Vector3d(1.0, 0.0, 0.0);
    State b;
    b.pos_i = Eigen::Vector3d(0.0, 1.0, 0.0);

    State sum = a + b;

    EXPECT_TRUE(sum.pos_i.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0)));
}
