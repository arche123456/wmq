#include "rm_armor_finder/Armor.h"
#include "rm_armor_finder/helper.h"
namespace rm_armor_finder {
    void ArmorPt::set(const Values &val, const bool &is_lg, const cv::RotatedRect &box1, const cv::RotatedRect &box2) {
        using namespace armor_helper;
        // box1点
        cv::Point2f pt_box1[4];
        box1.points(pt_box1);
        int min_1 = getMinPt(pt_box1);
        light_pt[1] = (pt_box1[0] + pt_box1[min_1]) / 2;
        light_pt[0] = (pt_box1[min_1 == 1 ? 2 : 1] + pt_box1[min_1 == 3 ? 2 : 3]) / 2;
        bool flip_box1 = false;
        if (light_pt[0].y > light_pt[1].y) {
            std::swap(light_pt[0], light_pt[1]);
            flip_box1 = true;
        }

        // box2点
        cv::Point2f pt_box2[4];
        box2.points(pt_box2);
        int min_2 = getMinPt(pt_box2);
        light_pt[2] = (pt_box2[0] + pt_box2[min_2]) / 2;
        light_pt[3] = (pt_box2[min_2 == 1 ? 2 : 1] + pt_box2[min_2 == 3 ? 2 : 3]) / 2;
        bool flip_box2 = false;
        if (light_pt[2].y < light_pt[3].y) {
            std::swap(light_pt[2], light_pt[3]);
            flip_box2 = true;
        }

        //左右互换
        auto pt_left_center = (light_pt[0] + light_pt[1]) / 2;
        auto pt_right_center = (light_pt[2] + light_pt[3]) / 2;
        center = (pt_left_center + pt_right_center) / 2;
        bool is_flip = pt_left_center.x > pt_right_center.x;
        if (is_flip) {
            std::swap(light_pt[0], light_pt[3]);
            std::swap(light_pt[1], light_pt[2]);
            std::swap(pt_left_center, pt_right_center);
        }

        //inner
        inner_pt[is_flip ? 3 : 0] = selectPt(!flip_box1, is_flip, min_1, pt_box1);
        inner_pt[is_flip ? 2 : 1] = selectPt(flip_box1, is_flip, min_1, pt_box1);
        inner_pt[is_flip ? 0 : 3] = selectPt(!flip_box2, !is_flip, min_2, pt_box2);
        inner_pt[is_flip ? 1 : 2] = selectPt(flip_box2, !is_flip, min_2, pt_box2);


        //number
        const auto pt_top_center = (light_pt[0] + light_pt[3]) / 2;
        const auto ratio_w = val.arm_num_w / (is_lg ? val.arm_lg_w : val.arm_sm_w);
        const auto reduce_w = (center - pt_left_center) * (1 - ratio_w);// 宽度缩小量
        const auto ratio_h = (is_lg ? val.arm_num_lg_h : val.arm_num_sm_h) / val.arm_l_h;
        const auto reduce_h = (center - pt_top_center) * (1 - ratio_h);//高度缩小量
        number_pt[0] = light_pt[0] + reduce_w + reduce_h;
        number_pt[1] = light_pt[1] + reduce_w - reduce_h;
        number_pt[2] = light_pt[2] - reduce_w - reduce_h;
        number_pt[3] = light_pt[3] - reduce_w + reduce_h;
    }
    float ArmorPt::dot(const cv::Point2f &p) const {
        return armor_helper::SquareDistance(center, p);
    }

    //计算面积
    float ArmorPt::area() const {
        return armor_helper::triangleArea(light_pt[0], light_pt[1], light_pt[2]) +
               armor_helper::triangleArea(light_pt[0], light_pt[2], light_pt[3]);
    }

    LightBar LightBar::getLightBar(cv::RotatedRect rr) {
        rr.center *= 2;
        rr.size.width *= 2;
        rr.size.height *= 2;
        auto angle = af_helper::getRealAngle(rr);
        auto w = rr.size.width, h = rr.size.height;
        auto area = rr.size.area();
        if (w > h) std::swap(w, h);
        return {
                rr,
                angle,
                w,
                h,
                h / w,
                area,
        };
    }
}// namespace rm_armor_finder