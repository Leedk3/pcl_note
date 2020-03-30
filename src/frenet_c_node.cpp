#include <ros/ros.h>
#include <tuple>
#include <typeinfo>
// #include "frenet_c/quintic_polynomials.hpp"
#include "frenet_c/QuarticPolynomial.hpp"
#include "frenet_c/FrenetPath.hpp"
#include "frenet_c/cubic_spliner.hpp"
#include "frenet_c/cubic_spliner_2D.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_c"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated frenet_c node");
    ros::Rate r(1);
    vector<double> wx,wy;
    wx.push_back(0.0);wx.push_back(10.0);wx.push_back(20.5);wx.push_back(30.0);wx.push_back(7000.5);
    wy.push_back(0.0);wy.push_back(-6.0);wy.push_back(5.0);wy.push_back(4.5);wy.push_back(0.0);

    vector<vector<double> > ob{ {-1110.0, 0.0}, 
                               {20.5, 5.0}, 
                               {30.0, 4.5},
                               {10.0, -6.0},
                               {-1110.0, 0.0} };

    vector<double> tx;

    vector<double> ty;
    vector<double> tyaw;
    vector<double> tc;
    Spline2D csp;

    ros::Publisher fp_vis_pub = n.advertise<visualization_msgs::MarkerArray>("frenet_path", 10);
    ros::Publisher target_vis_pub = n.advertise<visualization_msgs::Marker>("target_path", 10);
    ros::Publisher obstacle_vis_pub = n.advertise<visualization_msgs::Marker>("obstacle", 10);

    visualization_msgs::Marker fp, tp, obs;
    visualization_msgs::MarkerArray fp_all;
    fp.header.frame_id = "base_footprint";                  tp.header.frame_id = "base_footprint";                              obs.header.frame_id = "base_footprint";                  
    fp.header.stamp = ros::Time();                          tp.header.stamp = ros::Time();                                      obs.header.stamp = ros::Time();
    fp.id = 0;                                              tp.id = 0;                                                          obs.id = 0;
    fp.type = visualization_msgs::Marker::LINE_STRIP;       tp.type = visualization_msgs::Marker::LINE_STRIP;                   obs.type = visualization_msgs::Marker::SPHERE_LIST;
    fp.action = visualization_msgs::Marker::ADD;            tp.action = visualization_msgs::Marker::ADD;                        obs.action = visualization_msgs::Marker::ADD;
    fp.pose.position.x = 0;                                 tp.pose.position.x = 0;                                             obs.pose.position.x = 0;
    fp.pose.position.y = 0;                                 tp.pose.position.y = 0;                                             obs.pose.position.y = 0;
    fp.pose.orientation.w = 1.0;                            tp.pose.orientation.w = 1.0;                                        obs.pose.orientation.w = 1.0;
    fp.scale.x = 0.1;                                       tp.scale.x = 0.5;                                                   obs.scale.x = 0.1;
    fp.scale.y = 0.5;                                       tp.scale.y = 0.5;                                                   obs.scale.y = 0.1;
    fp.scale.z = 0.5;                                       tp.scale.z = 0.5;                                                   obs.scale.z = 0.1;
    fp.color.a = 1.0;                                       tp.color.a = 1.0;                                                   obs.color.a = 1.0; // Don't forget to set the alpha!
    fp.color.r = 0.0;                                       tp.color.r = 1.0;                                                   obs.color.r = 0.0;
    fp.color.g = 1.0;                                       tp.color.g = 0.0;                                                   obs.color.g = 0.0;
    fp.color.b = 0.0;                                       tp.color.b = 0.0;                                                   obs.color.b = 1.0;

    // vector of visual marker for path candidates



    std::tie(tx,ty,tyaw,tc,csp) = generate_target_course(wx,wy);

    double c_speed = 10.0 / 3.6;
    double c_d = 2.0;
    double c_d_d = 0.0;
    double c_d_dd = 0.0;
    double s0 = 0.0;

    double area = 20.0;

    for(int i = 0; i < int(tx.size()); i += 10) {
        geometry_msgs::Point p;
        p.x = tx[i];
        p.y = ty[i];
        p.z = 0;
        tp.points.push_back(p);
    }
    for(int i = 0; i < int(ob.size()); i ++) {
        geometry_msgs::Point p;
        p.x = ob[i][0];
        p.y = ob[i][1];
        p.z = 0;
        obs.points.push_back(p);
    }

    while(ros::ok()){

        for(int sim = 0; sim < 200; sim++) {
            FrenetPath path;
            vector<FrenetPath> path_all;

            // auto path = frenet_optimal_planning(n, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);
            std::tie(path, path_all) = frenet_optimal_planning(n, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);

            if(path.s[0] == -1 && path.d[0] == -1.1){
                cout << "All path are canceled, no progress" << endl;
            }
            else{
                double update_gain = 1;
                cout << "Optimal path is found, further progress" << endl;
                s0 = path.s[1] * update_gain;
                c_d = path.d[1] * update_gain;
                c_d_d = path.d_d[1] * update_gain;
                c_d_dd = path.d_dd[1] * update_gain;
                c_speed = path.s_d[1] * update_gain;

                // Visualization
                if(VISUALIZATION){
                    
                    fp_all.markers.clear();
                    for(int j = 0; j < int(path_all.size()); j++) {
                        visualization_msgs::Marker fp;
                        fp.header.frame_id = "base_footprint";
                        fp.header.stamp = ros::Time();
                        fp.id = j;
                        fp.type = visualization_msgs::Marker::LINE_STRIP;
                        fp.action = visualization_msgs::Marker::ADD;       
                        fp.pose.position.x = 0;            
                        fp.pose.position.y = 0;                            
                        fp.pose.orientation.w = 1.0;
                        fp.scale.x = 0.1;
                        fp.scale.y = 0.5;
                        fp.scale.z = 0.5;
                        fp.color.a = 1.0;
                        fp.color.r = 0.0;
                        fp.color.g = 1.0 * (j+1) / path_all.size();
                        fp.color.b = 0.0;

                        for(int i = 0; i < int(path_all[j].x.size()); i++) {
                            geometry_msgs::Point p;
                            p.x = path_all[j].x[i];
                            p.y = path_all[j].y[i];
                            p.z = 0;
                            fp.points.push_back(p);
                        }
                        fp_all.markers.push_back(fp);
                    }
                    
                    fp_vis_pub.publish(fp_all);
                    target_vis_pub.publish(tp);
                    obstacle_vis_pub.publish(obs);
                }
            }

            // sleep(10);
        }

        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}