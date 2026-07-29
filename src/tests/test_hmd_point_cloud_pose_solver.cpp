#include "HMDPointCloudPoseSolver.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
    static bool check(
        const bool condition,
        const char *expression,
        const char *test_name,
        const int line)
    {
        if (!condition)
        {
            std::fprintf(
                stderr,
                "%s:%d: %s failed: %s\n",
                test_name,
                line,
                test_name,
                expression);
        }
        return condition;
    }

#define TEST_CHECK(test_name, expression) \
    do \
    { \
        if (!check((expression), #expression, test_name, __LINE__)) \
        { \
            return false; \
        } \
    } while (false)

    static std::vector<cv::Point3f> make_model_points()
    {
        // Monado's approximate seven-front-light model, converted to cm.
        std::vector<cv::Point3f> points;
        points.push_back(cv::Point3f(-6.502f, 4.335f, 1.861f));
        points.push_back(cv::Point3f(6.502f, 4.335f, 1.861f));
        points.push_back(cv::Point3f(0.f, 0.f, 4.533f));
        points.push_back(cv::Point3f(-6.502f, -4.335f, 1.861f));
        points.push_back(cv::Point3f(6.502f, -4.335f, 1.861f));
        points.push_back(cv::Point3f(-7.802f, 0.f, -2.671f));
        points.push_back(cv::Point3f(7.802f, 0.f, -2.671f));
        return points;
    }

    static std::vector<cv::Point3f> make_service_bootstrap_model_points()
    {
        // Keep this fixture in the same E,C,F,A,D,G,B order as the default
        // MorpheusHMDConfig front-light profile.
        std::vector<cv::Point3f> points;
        points.push_back(cv::Point3f(0.f, 0.f, 0.f));
        points.push_back(cv::Point3f(7.25f, 4.05f, 3.75f));
        points.push_back(cv::Point3f(9.05f, 0.f, 9.65f));
        points.push_back(cv::Point3f(7.25f, -4.05f, 3.75f));
        points.push_back(cv::Point3f(-7.25f, 4.05f, 3.75f));
        points.push_back(cv::Point3f(-9.05f, 0.f, 9.65f));
        points.push_back(cv::Point3f(-7.25f, -4.05f, 3.75f));
        return points;
    }

    static cv::Matx33d make_camera_matrix()
    {
        return cv::Matx33d(
            900., 0., 640.,
            0., 900., 360.,
            0., 0., 1.);
    }

    static cv::Matx33d make_negative_fy_camera_matrix()
    {
        return cv::Matx33d(
            900., 0., 640.,
            0., -900., 360.,
            0., 0., 1.);
    }

    static cv::Vec<double, 5> make_distortion()
    {
        return cv::Vec<double, 5>(0., 0., 0., 0., 0.);
    }

    static cv::Vec<double, 5> make_nonzero_distortion()
    {
        return cv::Vec<double, 5>(
            -0.24,
            0.08,
            0.002,
            -0.0015,
            0.015);
    }

    static std::vector<cv::Point2f> project_model(
        const std::vector<cv::Point3f> &model_points,
        const cv::Vec3d &orientation_rvec,
        const cv::Vec3d &position_cm,
        const cv::Matx33d &camera_matrix = make_camera_matrix(),
        const cv::Vec<double, 5> &distortion_coefficients =
            make_distortion())
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            model_points,
            orientation_rvec,
            position_cm,
            camera_matrix,
            distortion_coefficients,
            projected);
        return projected;
    }

    static double rotation_error_degrees(
        const cv::Vec3d &left,
        const cv::Vec3d &right)
    {
        cv::Mat left_rotation;
        cv::Mat right_rotation;
        cv::Rodrigues(left, left_rotation);
        cv::Rodrigues(right, right_rotation);
        const cv::Mat delta =
            left_rotation * right_rotation.t();
        const double cosine =
            std::max(
                -1.,
                std::min(
                    1.,
                    (cv::trace(delta)[0] - 1.) * 0.5));
        return std::acos(cosine) * (180. / CV_PI);
    }

    static bool diagnostics_are_bounded(
        const HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        return
            diagnostics.orientation_hypothesis_count <=
                HMDPointCloudPoseSolver::
                    kMaxOrientationHypothesisCount &&
            diagnostics.pair_hypothesis_count <=
                HMDPointCloudPoseSolver::
                    kMaxRawPairHypothesisCount &&
            diagnostics.retained_coarse_candidate_count <=
                HMDPointCloudPoseSolver::
                    kMaxCoarseCandidateCount &&
            diagnostics.exact_assignment_count <=
                HMDPointCloudPoseSolver::
                    kMaxCoarseCandidateCount &&
            diagnostics.unique_mapping_count <=
                HMDPointCloudPoseSolver::
                    kMaxMappingHypothesisCount &&
            diagnostics.pnp_attempt_count <=
                HMDPointCloudPoseSolver::
                    kMaxPnPAttemptCount &&
            diagnostics.retained_pose_candidate_count <=
                HMDPointCloudPoseSolver::
                    kMaxPoseCandidateCount;
    }

    static HMDPointCloudPosePrior make_orientation_prior(
        const cv::Vec3d &orientation_rvec)
    {
        HMDPointCloudPosePrior prior;
        prior.orientation_rvec = orientation_rvec;
        prior.has_orientation = true;
        return prior;
    }

    static bool test_unordered_points_and_determinism()
    {
        const char *test_name =
            "unordered_points_and_determinism";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.08, -0.12, 0.03);
        const cv::Vec3d position_cm(4., -2., 110.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(cv::Point2f(90.f, 110.f));
        observed.push_back(projected[6]);
        observed.push_back(projected[2]);
        observed.push_back(cv::Point2f(1110.f, 620.f));
        observed.push_back(projected[0]);
        observed.push_back(projected[5]);
        observed.push_back(cv::Point2f(1040.f, 95.f));
        observed.push_back(projected[1]);
        observed.push_back(projected[4]);
        observed.push_back(projected[3]);

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult first_result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                first_result));
        TEST_CHECK(
            test_name,
            first_result.inlier_pairs.size() == 7);
        TEST_CHECK(
            test_name,
            cv::norm(first_result.position_cm - position_cm) < 0.5);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                first_result.orientation_rvec,
                orientation_rvec) < 0.5);
        TEST_CHECK(
            test_name,
            first_result.reprojection_error_px < 0.1);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(first_result.diagnostics));

        for (int repeat_index = 0;
             repeat_index < 3;
             ++repeat_index)
        {
            HMDPointCloudPoseResult repeated_result;
            TEST_CHECK(
                test_name,
                HMDPointCloudPoseSolver::solve(
                    model_points,
                    observed,
                    make_camera_matrix(),
                    make_distortion(),
                    prior,
                    settings,
                    repeated_result));
            TEST_CHECK(
                test_name,
                cv::norm(
                    repeated_result.position_cm -
                    first_result.position_cm) < 1e-9);
            TEST_CHECK(
                test_name,
                cv::norm(
                    repeated_result.orientation_rvec -
                    first_result.orientation_rvec) < 1e-9);
            TEST_CHECK(
                test_name,
                repeated_result.inlier_pairs ==
                    first_result.inlier_pairs);
            TEST_CHECK(
                test_name,
                repeated_result.diagnostics.pnp_attempt_count ==
                    first_result.diagnostics.pnp_attempt_count);
        }

        std::reverse(observed.begin(), observed.end());
        HMDPointCloudPoseResult reversed_result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                reversed_result));
        TEST_CHECK(
            test_name,
            cv::norm(
                reversed_result.position_cm -
                first_result.position_cm) < 1e-6);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                reversed_result.orientation_rvec,
                first_result.orientation_rvec) < 1e-6);

        return true;
    }

    static bool test_missing_points_noise_and_extras()
    {
        const char *test_name =
            "missing_points_noise_and_extras";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(-0.05, 0.14, -0.025);
        const cv::Vec3d position_cm(-3., 1.5, 125.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(
            projected[5] + cv::Point2f(0.9f, -0.7f));
        observed.push_back(cv::Point2f(80.f, 620.f));
        observed.push_back(
            projected[0] + cv::Point2f(-0.8f, 0.5f));
        observed.push_back(
            projected[3] + cv::Point2f(0.4f, 0.8f));
        observed.push_back(cv::Point2f(1170.f, 80.f));
        observed.push_back(
            projected[2] + cv::Point2f(-0.4f, -0.6f));
        observed.push_back(cv::Point2f(1100.f, 650.f));
        observed.push_back(
            projected[1] + cv::Point2f(0.6f, 0.4f));

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.inlier_pairs.size() >= 5);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 3.);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 4.);
        TEST_CHECK(
            test_name,
            result.reprojection_error_px < 2.);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_service_bootstrap_model()
    {
        const char *test_name = "service_bootstrap_model";
        const std::vector<cv::Point3f> model_points =
            make_service_bootstrap_model_points();
        const cv::Vec3d orientation_rvec(0.06, -0.10, 0.025);
        const cv::Vec3d position_cm(2.5, -1.5, 115.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[5]);
        observed.push_back(cv::Point2f(100.f, 620.f));
        observed.push_back(projected[1]);
        observed.push_back(projected[6]);
        observed.push_back(projected[3]);
        observed.push_back(cv::Point2f(1150.f, 100.f));
        observed.push_back(projected[0]);
        observed.push_back(projected[4]);
        observed.push_back(projected[2]);

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(test_name, result.inlier_pairs.size() == 7);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 0.5);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 0.5);
        TEST_CHECK(test_name, diagnostics_are_bounded(result.diagnostics));
        return true;
    }

    static bool test_four_point_continuation()
    {
        const char *test_name =
            "four_point_continuation";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.04, -0.09, 0.02);
        const cv::Vec3d position_cm(2., -1., 115.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[6]);
        observed.push_back(projected[0]);
        observed.push_back(cv::Point2f(100.f, 100.f));
        observed.push_back(projected[3]);
        observed.push_back(projected[1]);

        HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        prior.position_cm =
            position_cm + cv::Vec3d(1.5, -0.5, 2.);
        prior.position_age_seconds = 1. / 30.;
        prior.has_position = true;

        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.inlier_pairs.size() == 4);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 1.);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 1.);
        TEST_CHECK(
            test_name,
            result.diagnostics.orientation_hypothesis_count == 1);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_negative_fy_camera_matrix()
    {
        const char *test_name =
            "negative_fy_camera_matrix";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.06, -0.11, 0.025);
        const cv::Vec3d position_cm(3., -1.5, 118.);
        const cv::Matx33d camera_matrix =
            make_negative_fy_camera_matrix();
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm,
                camera_matrix);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[4]);
        observed.push_back(cv::Point2f(75.f, 75.f));
        observed.push_back(projected[1]);
        observed.push_back(projected[6]);
        observed.push_back(projected[0]);
        observed.push_back(cv::Point2f(1180.f, 660.f));
        observed.push_back(projected[3]);
        observed.push_back(projected[5]);
        observed.push_back(projected[2]);

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                camera_matrix,
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.inlier_pairs.size() == 7);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 0.5);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 0.5);
        TEST_CHECK(
            test_name,
            result.reprojection_error_px < 0.1);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_rejects_fresh_mirrored_position_prior()
    {
        const char *test_name =
            "rejects_fresh_mirrored_position_prior";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.04, -0.08, 0.015);
        const cv::Vec3d position_cm(45., -2., 115.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[5]);
        observed.push_back(projected[1]);
        observed.push_back(projected[3]);
        observed.push_back(projected[6]);
        observed.push_back(projected[0]);
        observed.push_back(projected[4]);
        observed.push_back(projected[2]);

        HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        prior.position_cm = cv::Vec3d(-45., -2., 115.);
        prior.position_age_seconds = 1. / 30.;
        prior.has_position = true;

        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            !HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.diagnostics.orientation_hypothesis_count == 1);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_rejects_imu_inconsistent_orientation()
    {
        const char *test_name =
            "rejects_imu_inconsistent_orientation";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d true_orientation_rvec(0., 0., 0.);
        const cv::Vec3d position_cm(2., -1., 105.);
        const std::vector<cv::Point2f> observed =
            project_model(
                model_points,
                true_orientation_rvec,
                position_cm);

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(
                cv::Vec3d(CV_PI * 0.5, 0., 0.));
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            !HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.diagnostics.orientation_hypothesis_count ==
                HMDPointCloudPoseSolver::
                    kMaxOrientationHypothesisCount);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_nonzero_lens_distortion_recovery()
    {
        const char *test_name =
            "nonzero_lens_distortion_recovery";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.09, -0.16, 0.035);
        const cv::Vec3d position_cm(28., 12., 95.);
        const cv::Vec<double, 5> distortion_coefficients =
            make_nonzero_distortion();
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm,
                make_camera_matrix(),
                distortion_coefficients);
        const std::vector<cv::Point2f> undistorted_projection =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        double maximum_distortion_displacement_px = 0.;
        for (size_t point_index = 0;
             point_index < projected.size();
             ++point_index)
        {
            maximum_distortion_displacement_px =
                std::max(
                    maximum_distortion_displacement_px,
                    static_cast<double>(
                        cv::norm(
                            projected[point_index] -
                            undistorted_projection[point_index])));
        }
        TEST_CHECK(
            test_name,
            maximum_distortion_displacement_px > 2.);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[4]);
        observed.push_back(cv::Point2f(70.f, 630.f));
        observed.push_back(projected[1]);
        observed.push_back(projected[6]);
        observed.push_back(projected[0]);
        observed.push_back(cv::Point2f(1140.f, 80.f));
        observed.push_back(projected[3]);
        observed.push_back(projected[5]);
        observed.push_back(projected[2]);

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                distortion_coefficients,
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.inlier_pairs.size() == 7);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 0.5);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 0.5);
        TEST_CHECK(
            test_name,
            result.reprojection_error_px < 0.1);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_stale_position_prior_reacquisition()
    {
        const char *test_name =
            "stale_position_prior_reacquisition";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        const cv::Vec3d orientation_rvec(0.04, -0.08, 0.015);
        const cv::Vec3d position_cm(45., -2., 115.);
        const std::vector<cv::Point2f> projected =
            project_model(
                model_points,
                orientation_rvec,
                position_cm);

        std::vector<cv::Point2f> observed;
        observed.push_back(projected[5]);
        observed.push_back(projected[1]);
        observed.push_back(projected[3]);
        observed.push_back(projected[6]);
        observed.push_back(projected[0]);
        observed.push_back(projected[4]);
        observed.push_back(projected[2]);

        HMDPointCloudPosePrior prior =
            make_orientation_prior(orientation_rvec);
        prior.position_cm = cv::Vec3d(-45., -2., 115.);
        prior.position_age_seconds = 1.;
        prior.has_position = true;

        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.diagnostics.orientation_hypothesis_count ==
                HMDPointCloudPoseSolver::
                    kMaxOrientationHypothesisCount);
        TEST_CHECK(
            test_name,
            result.inlier_pairs.size() == 7);
        TEST_CHECK(
            test_name,
            cv::norm(result.position_cm - position_cm) < 0.5);
        TEST_CHECK(
            test_name,
            rotation_error_degrees(
                result.orientation_rvec,
                orientation_rvec) < 0.5);
        TEST_CHECK(
            test_name,
            diagnostics_are_bounded(result.diagnostics));

        return true;
    }

    static bool test_rejects_unbounded_input()
    {
        const char *test_name =
            "rejects_unbounded_input";
        const std::vector<cv::Point3f> model_points =
            make_model_points();
        std::vector<cv::Point2f> observed;
        for (int point_index = 0;
             point_index <
                 HMDPointCloudPoseSolver::kMaxImagePointCount + 1;
             ++point_index)
        {
            observed.push_back(
                cv::Point2f(
                    static_cast<float>(20 + point_index * 10),
                    static_cast<float>(40 + point_index * 5)));
        }

        const HMDPointCloudPosePrior prior =
            make_orientation_prior(cv::Vec3d(0., 0., 0.));
        HMDPointCloudPoseSolverSettings settings;
        HMDPointCloudPoseResult result;
        TEST_CHECK(
            test_name,
            !HMDPointCloudPoseSolver::solve(
                model_points,
                observed,
                make_camera_matrix(),
                make_distortion(),
                prior,
                settings,
                result));
        TEST_CHECK(
            test_name,
            result.diagnostics.pair_hypothesis_count == 0);
        TEST_CHECK(
            test_name,
            result.diagnostics.pnp_attempt_count == 0);

        return true;
    }
}

int main()
{
    bool success = true;
    success =
        test_unordered_points_and_determinism() &&
        success;
    success =
        test_missing_points_noise_and_extras() &&
        success;
    success =
        test_service_bootstrap_model() &&
        success;
    success =
        test_four_point_continuation() &&
        success;
    success =
        test_negative_fy_camera_matrix() &&
        success;
    success =
        test_rejects_fresh_mirrored_position_prior() &&
        success;
    success =
        test_rejects_imu_inconsistent_orientation() &&
        success;
    success =
        test_nonzero_lens_distortion_recovery() &&
        success;
    success =
        test_stale_position_prior_reacquisition() &&
        success;
    success =
        test_rejects_unbounded_input() &&
        success;

    std::fprintf(
        stdout,
        "HMD point-cloud pose solver tests: %s\n",
        success ? "PASSED" : "FAILED");
    return success ? 0 : 1;
}
