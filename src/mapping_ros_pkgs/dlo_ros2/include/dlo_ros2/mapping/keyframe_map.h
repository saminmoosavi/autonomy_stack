// keyframe_map.h
//   declares KeyframeMap and private selection functions
#pragma once

#include <cstddef>
#include <vector>

#include "dlo_ros2/types/point_cloud.h"
#include "dlo_ros2/types/pose.h"
#include "dlo_ros2/interfaces/i_localmap.h"

namespace dlo {

class KeyFrameMap : public ILocalMap {
public:
    void addFrame(const PointCloud& cloud,
                  const Pose& pose) override;

    PointCloud getMap() const override;

private:
    std::vector<PointCloud> frames_;
    std::vector<Pose> poses_;
    std::size_t num_neighbors_= 5;
    dlo::Pose latest_pose_;

    std::vector<std::size_t> selectNearestNeighbor() const; //return which points to use
    std::vector<std::size_t> convexHull() const ;
    std::vector<std::size_t> concaveHull() const ;

};

}

// keyframe_map.cpp

//   addFrame()
//     decides whether current frame becomes a keyframe

//   getMap()
//     chooses keyframes using selection mode
//     transforms selected keyframe clouds into map frame
//     merges them into one PointCloud

//   selectNearestNeighbors()
//     chooses K closest keyframes

//   selectConvexHull()
//     chooses keyframes on spatial boundary

//   selectConcaveHull()
//     choose a tighter boundary that follows the shape of the trajectory/environment