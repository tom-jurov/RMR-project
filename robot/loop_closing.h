#ifndef SLAM_IN_AUTO_DRIVING_LOOP_CLOSING_H
#define SLAM_IN_AUTO_DRIVING_LOOP_CLOSING_H

#include "eigen_types.h"
#include "multi_resolution_likelihood_field.h"
#include "submap.h"

#include <fstream>
#include <map>
#include <memory>


class LoopClosing {
public:
    /// 一个回环约束
    struct LoopConstraints {
        LoopConstraints(size_t id1, size_t id2, const SE2& T12) : id_submap1_(id1), id_submap2_(id2), T12_(T12) {}
        size_t id_submap1_ = 0;
        size_t id_submap2_ = 0;
        SE2 T12_;  //  相对pose
        bool valid_ = true;
    };

    LoopClosing() { debug_fout_.open("./data/ch6/loops.txt"); }

    /// 添加最近的submap，这个submap可能正在构建中
    void AddNewSubmap(std::shared_ptr<Submap> submap);

    /// 添加一个已经建完的submap，需要在AddNewSubmap函数之后调用
    void AddFinishedSubmap(std::shared_ptr<Submap> submap);

    /// 为新的frame进行回环检测，更新它的pose和submap的pose
    void AddNewFrame(std::shared_ptr<Frame> frame);

    /// 获取submap之间的回环检测
    std::map<std::pair<size_t, size_t>, LoopConstraints> GetLoops() const { return loop_constraints_; }

    bool HasNewLoops() const { return has_new_loops_; }

private:
    /// 检测当前帧与历史地图可能存在的回环
    bool DetectLoopCandidates();

    /// 将当前帧与历史submap进行匹配
    void MatchInHistorySubmaps();

    /// 进行submap间的pose graph优化
    void Optimize();

    std::shared_ptr<Frame> current_frame_ = nullptr;
    size_t last_submap_id_ = 0;  // 最新一个submap的id

    std::map<size_t, std::shared_ptr<Submap>> submaps_;  // 所有的submaps

    // submap到mr field之间的对应关系
    std::map<std::shared_ptr<Submap>, std::shared_ptr<MRLikelihoodField>> submap_to_field_;

    std::vector<size_t> current_candidates_;                                 // 可能的回环检测点
    std::map<std::pair<size_t, size_t>, LoopConstraints> loop_constraints_;  // 回环约束, 以被约束的两个submap为索引
    bool has_new_loops_ = false;

    std::ofstream debug_fout_;  // 调试输出

    /// 参数
    inline static constexpr float candidate_distance_th_ = 15.0;  // candidate frame与submap中心之间的距离
    inline static constexpr int submap_gap_ = 1;                  // 当前scan与最近submap编号上的差异
    inline static constexpr double loop_rk_delta_ = 1.0;          // 回环检测的robust kernel 阈值
};

#endif  // SLAM_IN_AUTO_DRIVING_LOOP_CLOSING_H
