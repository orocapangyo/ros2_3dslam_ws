#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "mapper/wall_aligner_node.hpp"

/*
 * 주의: RANSAC 벽 탐지 테스트는 wall_detector 패키지
 * (test_wall_detector.cpp) 로 이전되었다.
 *
 * wall_aligner_node는 이제 /wall_detector/longest_wall 을 구독하므로
 * 탐지 로직 자체는 여기서 테스트하지 않는다.
 *
 * 본 파일은 노드 생성/파라미터 로드 스모크 테스트만 수행한다.
 * 정렬 루프 통합 테스트는 SIL(Gazebo) 환경에서 수행한다.
 */

class WallAlignerSmokeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()   { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(WallAlignerSmokeTest, NodeConstructsWithDefaults) {
    auto node = std::make_shared<mapper::WallAlignerNode>();
    EXPECT_EQ(std::string(node->get_name()), "wall_aligner_node");
}

TEST_F(WallAlignerSmokeTest, ParametersAreDeclared) {
    auto node = std::make_shared<mapper::WallAlignerNode>();
    EXPECT_TRUE(node->has_parameter("tolerance_deg"));
    EXPECT_TRUE(node->has_parameter("max_attempts"));
    EXPECT_TRUE(node->has_parameter("cooldown_ms"));
    EXPECT_TRUE(node->has_parameter("min_inlier_ratio"));
    EXPECT_TRUE(node->has_parameter("wall_topic"));
}
