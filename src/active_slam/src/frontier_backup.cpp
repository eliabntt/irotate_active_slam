#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

double robot_inscribed_radius = 0.3;

ros::Publisher pub_vis;
ros::Publisher pub_frontier_cells;

struct FrontierPoint{
    int idx;     //position
    tf::Vector3 d; //direction

    FrontierPoint(int idx_, const tf::Vector3& d_) : idx(idx_), d(d_) {}
};

struct Frontier {
    geometry_msgs::Pose pose;
    int size;
    Frontier():pose(),size(0) {}
    Frontier(const Frontier& copy) { pose = copy.pose; size = copy.size; }
};

void findFrontiers(const nav_msgs::OccupancyGrid& occ_grid, std::vector<Frontier>& frontiers) {
    frontiers.clear();

    int idx;
    int w = occ_grid.info.width;
    int h = occ_grid.info.height;
    int size = (w * h);

    std::vector<bool> frontier_map(size, false);

    nav_msgs::OccupancyGrid vis_frontier_cells;
    vis_frontier_cells.header = occ_grid.header;
    vis_frontier_cells.info = occ_grid.info;
    vis_frontier_cells.data.resize(size);

    // Find all frontier cells (free cells next to unknown cells).
    const std::vector<signed char>& map = occ_grid.data;
    for (idx = 0; idx < size; idx++) {
        bool free_cell = (map[idx] >= 0 && map[idx] < 50);

        if (free_cell &&
            (   (idx + 1 < size && map[idx + 1] < 0)
                || (idx - 1 >= 0 && map[idx - 1] < 0)
                || (idx + w < size && map[idx + w] < 0)
                || (idx - w >= 0 && map[idx - w] < 0))) {
            frontier_map[idx] = true;
            vis_frontier_cells.data[idx] = 0;
        } else {
            vis_frontier_cells.data[idx] = 100;
        }
    }

    pub_frontier_cells.publish(vis_frontier_cells);

    // Clean up frontiers detected on separate rows of the map - rollout of the array idx and idx+1 could be frontiers but in two different rows
    // of the map (hence on the opposite side)
    idx = h - 1;
    for (int y = 0; y < w; y++) {
        frontier_map[idx] = false;
        idx += h;
    }

    // Group adjoining frontier cells
    std::vector< std::vector<FrontierPoint> > segments;
    for (int i = 0; i < size; i++) {
        if (frontier_map[i]) {
            std::vector<int> neighbors;
            std::vector<FrontierPoint> segment;
            neighbors.push_back(i);

            // claim all neighbors
            while (neighbors.size() > 0) {
                int idx = neighbors.back();
                neighbors.pop_back();

                // make sure this frontier cell is not visited again
                frontier_map[idx] = false;

                tf::Vector3 dir(0, 0, 0);
                int c = 0;
                if (idx + 1 < size && map[idx+1] < 0) {
                    dir += tf::Vector3(1, 0, 0);
                    c++;
                }
                if (idx-1 >= 0 && map[idx-1] < 0) {
                    dir += tf::Vector3(-1, 0, 0);
                    c++;
                }
                if (idx+w < size && map[idx+w] < 0) {
                    dir += tf::Vector3(0, 1, 0);
                    c++;
                }
                if (idx-w >= 0 && map[idx-w] < 0) {
                    dir += tf::Vector3(0, -1, 0);
                    c++;
                }

                segment.push_back(FrontierPoint(idx, dir / c));

                // consider 8 neighborhood
                if (idx - 1 > 0 && frontier_map[idx - 1])
                    neighbors.push_back(idx - 1);

                if (idx + 1 < size && frontier_map[idx + 1])
                    neighbors.push_back(idx + 1);

                if (idx - w > 0 && frontier_map[idx - w])
                    neighbors.push_back(idx - w);

                if (idx - w + 1 > 0 && frontier_map[idx-w+1])
                    neighbors.push_back(idx - w + 1);

                if (idx - w - 1 > 0 && frontier_map[idx - w - 1])
                    neighbors.push_back(idx-w-1);

                if (idx + w < size && frontier_map[idx + w])
                    neighbors.push_back(idx+w);

                if (idx + w + 1 < size && frontier_map[idx + w + 1])
                    neighbors.push_back(idx + w + 1);

                if (idx + w - 1 < size && frontier_map[idx + w - 1])
                    neighbors.push_back(idx + w - 1);
            }

            segments.push_back(segment);
        }
    }

    for (unsigned int i = 0; i < segments.size(); i++) {
        Frontier frontier;
        std::vector<FrontierPoint>& segment = segments[i];
        uint size = segment.size();
        //we want to make sure that the frontier is big enough for the robot to fit through
        if (size * occ_grid.info.resolution >= robot_inscribed_radius) {
            float x = 0, y = 0;
            tf::Vector3 d(0,0,0);

            for (unsigned int j = 0; j < size; j++) {
                d += segment[j].d;
                int idx = segment[j].idx;
                x += (idx % w);
                y += (idx / w);
            }
            d = d / size;
            frontier.pose.position.x = occ_grid.info.origin.position.x + occ_grid.info.resolution * (x / size);
            frontier.pose.position.y = occ_grid.info.origin.position.y + occ_grid.info.resolution * (y / size);
            frontier.pose.position.z = 0.0;

            frontier.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(d.y(), d.x()));
            frontier.size = size;

            frontiers.push_back(frontier);
        }
    }
}

void visualizeFrontiers(const std::vector<Frontier>& frontiers) {
    visualization_msgs::MarkerArray markers;

    int id = 0;
    for(std::vector<Frontier>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it) {
        const Frontier& f = *it;
        visualization_msgs::Marker m;
        m.action = visualization_msgs::Marker::ADD;

        m.color.a = 1;
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 0;

        m.header.frame_id = "/map";
        m.header.stamp = ros::Time::now();

        m.id = id++;
        m.lifetime = ros::Duration(1.0);
        m.pose = f.pose;
        m.scale.x = 1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;

        markers.markers.push_back(m);
    }

    pub_vis.publish(markers);
}

void foo(const nav_msgs::OccupancyGrid& msg) {
    std::vector<Frontier> frontiers;
    findFrontiers(msg, frontiers);
    visualizeFrontiers(frontiers);
}

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    foo(*msg);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "frontier_backup");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map", 10, &callback);

    pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/explore/frontiers", 1);
    pub_frontier_cells = nh.advertise<nav_msgs::OccupancyGrid>("/explore/frontier_cells", 1);

    // subscribing to gmapping service (get map)
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/rtabmap/get_prob_map");

    ros::Rate r(1);
    while(ros::ok()) {
        ros::spinOnce();

        nav_msgs::GetMap srv;
        if (client.call(srv)) {
            foo(srv.response.map);
        } else {
            ROS_ERROR("Failed to call service");
        }

        r.sleep();
    }

    return 0;
}
