#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "freespace_detector_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define Z_WPT -0.6
#define ARRAY_SIZE 100

Freespace_detector::Freespace_detector(): Node("freespace_detector_node")
{

    // タイマー実行されるイベントハンドラー関数
    auto depict_waypoint =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {

    };
}

void Freespace_detector::cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);

    qgl_deg_ = yaw_*M_RAD2DEG;
    xgl_ = msg->pose.pose.position.x;
    ygl_ = msg->pose.pose.position.y;

    double distWpt = Dist_point_to_point(dq_Waypoint_[cur_wpt_num_].xgl,dq_Waypoint_[cur_wpt_num_].ygl,xgl_,ygl_);
    //printf("WPS::distWpt=%lf, Wpt_change_radius = %lf\n",distWpt,Wpt_change_radius_);
   
}

//点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
double Freespace_detector::Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2){

	double dist=0;
	double tmp1,tmp2;
	double q_rad;

	if(fabs(x2-x1) < VERY_SMALL_VAL){
		return -1;
	}

	q_rad = atan2(y2-y1,x2-x1);

	double a = tan(q_rad);
	double b = -1;
	double c = y1 - a*x1;

	tmp1 = a*xp+b*yp+c;
	tmp2 = sqrt(a*a+b*b);

	if(fabs(tmp2) < VERY_SMALL_VAL) tmp2 = VERY_SMALL_VAL;
	dist = fabs(tmp1/tmp2);

	return dist;
}

//点と点の距離
double Freespace_detector::Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y){
  double dist =0;

  dist = (p1_x-p2_x)*(p1_x-p2_x) + (p1_y-p2_y)*(p1_y-p2_y);
  dist = sqrt(dist);

  return dist;
}



int Freespace_detector::cb_Target_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

    pcl::fromROSMsg(*cloud_msg, PointcloudTarget_pcl_);
    //printf("WP:cb_Target_Pointcloud\n");
/*
    if(Detect_Object() > 0){
        Search_Lane();
        Set_LineFollowPoint();
        Depict_EgoRegion();
    }
*/
    return 1;
}

int  Freespace_detector::Detect_Object(void){

    static int count = 0;
    count++;
    visualization_msgs::msg::MarkerArray marker_array;

    if(PointcloudTarget_pcl_.makeShared()==nullptr) return 0;

    // KD木を作っておく。近傍点探索とかが早くなる。
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    if(PointcloudTarget_pcl_.size()<=0) return 0;

    tree->setInputCloud(PointcloudTarget_pcl_.makeShared());

    //近傍点探索に使うパラメータと結果が入る変数
    double radius = 5.0;  //半径r
    std::vector<int> k_indices;  //範囲内の点のインデックスが入る
    std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
    unsigned int max_nn = 100;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない

    pcl::PointXYZ p;  //中心座標
    p.x = 0;
    p.y = 0;
    p.z = 0;

    //半径r以内にある点を探索
    k_indices.clear();
    tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);

    if(k_indices.size() == 0) return 0;

    pcl::PointXYZ point = PointcloudTarget_pcl_.points[k_indices[0]];
    pcl::PointXYZ closest_point;
    point.z = 0;

    double min_dist = 9999;
    double dist = 0;
    for(uint k=0;k < k_indices.size();k++ ){
        point = PointcloudTarget_pcl_.points[k_indices[k]];

        if(point.z > MinHeight_){
            if(point.x > 0.1 && point.x < 5.0 && point.y > -AvoidanceMergin_ && point.y < AvoidanceMergin_ ){
                dist = Dist_point_to_point(0,0,point.x,point.y);
                if(dist < min_dist){
                    closest_point = point;
                    min_dist = dist;
                }
            }
        }
        //printf("point[%d] of (%lf, %lf) -> x: %lf, y:%lf, z:%lf \n", k, p.x, p.y, result.x, result.y, result.z);
    }

    distObject_ = min_dist;
    //closest_point

    return 1;
}


int Freespace_detector::Search_Lane(void){

    //printf("WP: Search_Lane():1\n");

    // KD木を作っておく。近傍点探索とかが早くなる。
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud(PointcloudTarget_pcl_.makeShared());

    //printf("WP: Search_Lane():2\n");

    //近傍点探索に使うパラメータと結果が入る変数
    double radius = 8.0;  //半径r
    std::vector<int> k_indices;  //範囲内の点のインデックスが入る
    std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
    unsigned int max_nn = 1000;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    vector<point3D> vec_Object;
    point3D ptObj;

    pcl::PointXYZ pt;
    pcl::PointXYZ center_pt;  //中心座標

    pcl::PointXYZ p;  //中心座標
    p.x = 0;p.y = 0;p.z = 0;

    //半径r以内にある点を探索
    k_indices.clear();
    tree->nearestKSearch(p, max_nn, k_indices, k_sqr_distances);
    //tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);

    //printf("WP: Search_Lane():3\n");

    if(k_indices.size() == 0) return 0;
    ////

    double yawWpt=0;

    double cos_gl;
    double sin_gl;

    double xwpt_fs=0, ywpt_fs=0, zwpt_fs=0;

    double x2wpt_fs=0, y2wpt_fs=0, z2wpt_fs=0;
    double x1wpt_fs=0, y1wpt_fs=0, z1wpt_fs=0;

    double x1org=0, y1org=0;
    double x2org=0, y2org=0;

    double x1=0, y1=0;
    double x2=0, y2=0;

    double x1org_fs=0, y1org_fs=0;
    double x2org_fs=0, y2org_fs=0;

    double x1fs=0, y1fs=0;
    double x2fs=0, y2fs=0;

    double dist_p2p=0;

    //printf("WP: Search_Lane():4\n");
    if(cur_wpt_num_ > 0 && cur_wpt_num_ < wpt_size_-1){

        x1org = prev_wpt_.x0gl-xgl_;
        y1org = prev_wpt_.y0gl-ygl_;

        x2org = current_wpt_.x0gl-xgl_;
        y2org = current_wpt_.y0gl-ygl_;

        x1 = prev_wpt_.xgl-xgl_;
        y1 = prev_wpt_.ygl-ygl_;

        x2 = current_wpt_.xgl-xgl_;
        y2 = current_wpt_.ygl-ygl_;

        yawWpt = atan2(y2-y1,x2-x1);
        cos_gl = cos(yaw_);
        sin_gl = sin(yaw_);

        x1org_fs = cos_gl*x1org + sin_gl*y1org;
        y1org_fs = -sin_gl*x1org + cos_gl*y1org;

        x2org_fs = cos_gl*x2org + sin_gl*y2org;
        y2org_fs = -sin_gl*x2org + cos_gl*y2org;

        x1fs = cos_gl*x1 + sin_gl*y1;
        y1fs = -sin_gl*x1 + cos_gl*y1;

        x2fs = cos_gl*x2 + sin_gl*y2;
        y2fs = -sin_gl*x2 + cos_gl*y2;

    }

    //FILE* F_IDX = fopen("indices.dat","w");
    //FILE* F_OBJ = fopen("object.dat","w");
    //FILE* F_WPT = fopen("waypoint.dat","w");

    double distWpt = sqrt(x2*x2+y2*y2) + 1.0;
    double minFrontDist = 9999;
    double tempFrontDist = 0;
    pcl::PointXYZ tempMinPt;

    vec_Object.clear();

    for(uint k=0;k < k_indices.size();k++ ){

        if(k_indices[k] > PointcloudTarget_pcl_.points.size()-1){
            continue;
        }

        pt = PointcloudTarget_pcl_.points[k_indices[k]];

        if(pt.x < AvoidanceArea_Back_ || pt.x > AvoidanceArea_Front_ || pt.y < -AvoidanceArea_Side_ || pt.y > AvoidanceArea_Side_ ) continue;
        if(pt.x > 0.05 && pt.x < AvoidanceArea_Front_ && pt.y > -AvoidanceMergin_ && pt.y < AvoidanceMergin_ && pt.z > MinHeight_ && pt.z < 0) {
            tempFrontDist = sqrt(pt.x*pt.x + pt.y*pt.y);
            if(minFrontDist > tempFrontDist){
                minFrontDist = tempFrontDist;
                tempMinPt = pt;
            }
        }

        dist_p2p = fabs(Dist_from_point_to_line2(pt.x,pt.y,x1fs,y1fs,x2fs,y2fs));
        //fprintf(F_OBJ,"%lf %lf %lf\n",pt.x,pt.y,pt.z);
        ptObj.x = pt.x;
        ptObj.y = pt.y;
        ptObj.z = pt.z;

        vec_Object.push_back(ptObj);
    }

    minFrontPt_ = tempMinPt;
    minFrontDist_ = minFrontDist;
    //printf("WP:minFrontDist_=%lf, (%lf,%lf) \n",minFrontDist_,minFrontPt_.x,minFrontPt_.y);

    //printf("WP: k_indices.size()=%d, NumObjectPt=%d\n",k_indices2.size(),vec_Object.size());

    int NumObjectPt = 0;
    int NumObjectPtL = 0;
    int NumObjectPtR = 0;
    double tmp_xAdjWpt = xAdjWpt_;
    double tmp_yAdjWptL = 0;
    double tmp_yAdjWptR = 0;


    if(vec_Object.size()>0){

        //Original path
        for(uint k=0;k < vec_Object.size();k++ ){
            ptObj = vec_Object[k];
            dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1org_fs,y1org_fs,x2org_fs,y2org_fs));

            if(ptObj.x > 0.1 && ptObj.x < distWpt && dist_p2p < AvoidanceMergin_){
                NumObjectPt++;
                break;
            }
        }

        if(NumObjectPt > 0){

            for(uint i=0;i < ObjectMonitorWigth_cm_;i++ ){ //1[m]
                //Right
                tmp_yAdjWptR = -0.01*i;
                NumObjectPtR = 0;
                for(uint k=0;k < vec_Object.size();k++ ){

                    ptObj = vec_Object[k];
                    dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs+xAdjWpt_,y1fs+tmp_yAdjWptR,x2fs+xAdjWpt_,y2fs+tmp_yAdjWptR));

                    if(ptObj.x > AvoidMinDist_ && ptObj.x < distWpt+1.0 && dist_p2p < AvoidanceMergin_){
                        NumObjectPtR ++;
                        break;
                    }
                }
                if(NumObjectPtR == 0){
                    //printf("WP:Right(%d)=%lf\n",i,dist_p2p);
                    xAdjWpt_ = cos_gl*tmp_xAdjWpt - sin_gl*tmp_yAdjWptR;
                    yAdjWpt_ = sin_gl*tmp_xAdjWpt + cos_gl*tmp_yAdjWptR;

                    break;
                }

                //Left
                tmp_yAdjWptL = 0.01*i;
                NumObjectPtL = 0;
                for(uint k=0;k < vec_Object.size();k++ ){

                    ptObj = vec_Object[k];
                    dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs+xAdjWpt_,y1fs+tmp_yAdjWptL,x2fs+xAdjWpt_,y2fs+tmp_yAdjWptL));

                    if(ptObj.x > AvoidMinDist_ && ptObj.x < distWpt+1.0 && dist_p2p < AvoidanceMergin_){
                        NumObjectPtL ++;
                        break;
                    }
                }

                if(NumObjectPtL == 0){
                    //printf("WP:Left(%d)=%lf\n",i,dist_p2p);
                    xAdjWpt_ = cos_gl*tmp_xAdjWpt - sin_gl*tmp_yAdjWptL;
                    yAdjWpt_ = sin_gl*tmp_xAdjWpt + cos_gl*tmp_yAdjWptL;

                    break;
                }
            }
        }
    }
    if(NumObjectPt == 0){
        wpt_modifyOrder_ = 1;
    }
    else{
        wpt_modifyOrder_ = 2;
    }

    //Extract Aoid Target
    if(cur_wpt_num_ > 0 && cur_wpt_num_ < wpt_size_-1){

        x1 = prev_wpt_.xgl-xgl_;
        y1 = prev_wpt_.ygl-ygl_;

        x2 = current_wpt_.xgl-xgl_;
        y2 = current_wpt_.ygl-ygl_;


        yawWpt = atan2(y2-y1,x2-x1);
        cos_gl = cos(yaw_);
        sin_gl = sin(yaw_);

        x1fs = cos_gl*x1 + sin_gl*y1;
        y1fs = -sin_gl*x1 + cos_gl*y1;

        x2fs = cos_gl*x2 + sin_gl*y2;
        y2fs = -sin_gl*x2 + cos_gl*y2;

        double tmpMinDist = 9999;
        double tmpDist = 9999;
        double tmpAvoidanceMergin = AvoidanceMergin_ + 0.25;
        pcl::PointXYZ tmpMinPt;

        for(uint k=0;k < vec_Object.size();k++ ){
            ptObj = vec_Object[k];
            dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs,y1fs,x2fs,y2fs));

            if(ptObj.x > 0.1 && ptObj.x < distWpt && dist_p2p < tmpAvoidanceMergin){

                tmpDist = Dist_point_to_point(0,0,ptObj.x,ptObj.y);

                if(ptObj.x > 0 && ptObj.z > Z_WPT + 0.1 && tmpMinDist > tmpDist){
                    tmpMinDist = tmpDist;
                    tmpMinPt.x = ptObj.x;
                    tmpMinPt.y = ptObj.y;
                    tmpMinPt.z = ptObj.z;
                }
             }
        }

        if(tmpMinDist == 9999){
            distAoid_ = distWpt + 1.5;
            avoidPt_.x = x2;
            avoidPt_.y = y2;
            avoidPt_.z = Z_WPT;
        }
        else{
            distAoid_ = tmpMinDist;
            avoidPt_ = tmpMinPt;
        }
    }
    return 1;
}

int Freespace_detector::Depict_EgoRegion(void){

    geometry_msgs::msg::Point p1;
    double x1,y1,x2,y2;
    double xp,yp,xp1,yp1,xp2,yp2;
    double xpp1,ypp1,xpp2,ypp2;
    double as,xs1,ys1,xs2,ys2;
    //crossing point;
    double xcp;
    double ycp;

    double d_region = EgoRegionLength_;
    double cos_gl = cos(yaw_);
    double sin_gl = sin(yaw_);

    x1 = xp = xgl_;
    y1 = yp = ygl_;

    x2 =  d_region*cos_gl + x1;
    y2 =  d_region*sin_gl + y1;

    //Lane
    //点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
    Calc_CrossPoint4viz(x1,y1,x2,y2,xp,yp,&xcp,&ycp);
    Calc_rect4viz(AvoidanceMergin_,xcp,ycp,x2,y2,&xp1,&yp1,&xpp1,&ypp1);
    Calc_rect4viz(AvoidanceMergin_,x2,y2,x1,y1,&xp2,&yp2,&xpp2,&ypp2);

    ego_pp1_.x = xp2;
    ego_pp1_.y = yp2;
    ego_pp1_.z = Z_WPT;

    ego_pp2_.x = xp1;
    ego_pp2_.y = yp1;
    ego_pp2_.z = Z_WPT;

    ego_pp3_.x = xpp1;
    ego_pp3_.y = ypp1;
    ego_pp3_.z = Z_WPT;

    ego_pp4_.x = xpp2;
    ego_pp4_.y = ypp2;
    ego_pp4_.z = Z_WPT;


    return 1;
}


int Freespace_detector::Set_LineFollowPoint(void){

    static int count = 0;
    geometry_msgs::msg::Point p1;
    double x1,y1,x2,y2;
    double xp,yp,xp1,yp1,xp2,yp2;
    double xpp1,ypp1,xpp2,ypp2;
    double as,xs1,ys1,xs2,ys2;
    //crossing point;
    double xcp;
    double ycp;

    double cos_gl = cos(yaw_);
    double sin_gl = sin(yaw_);

    x1 =  current_wpt_.l_xgl;
    y1 =  current_wpt_.l_ygl;

    x2 =  current_wpt_.xgl;
    y2 =  current_wpt_.ygl;

    xp = xgl_;
    yp = ygl_;

    //Lane
    //点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
    Calc_CrossPoint4viz(x1,y1,x2,y2,xp,yp,&xcp,&ycp);
    Calc_rect4viz(AvoidanceMergin_,xcp,ycp,x2,y2,&xp1,&yp1,&xpp1,&ypp1);
    Calc_rect4viz(AvoidanceMergin_,x2,y2,x1,y1,&xp2,&yp2,&xpp2,&ypp2);

    //FollowPoint
    if(fabs(x2-x1) > 0){

        double d = min(distAoid_-1.0,3.0);

        if(d < 0.1) d = 0.1;

        as = (y2-y1)/(x2-x1);
        xs1 = xcp + d/sqrt(as*as+1);
        xs2 = xcp - d/sqrt(as*as+1);
        ys1 = as*(xs1 - xcp) + ycp;
        ys2 = as*(xs2 - xcp) + ycp;

        double dist1 = Dist2_point_to_point(x2,y2,xs1,ys1);
        double dist2 = Dist2_point_to_point(x2,y2,xs2,ys2);

        if(dist1 < dist2) {
            xfollowWptGL_ = xs1;
            yfollowWptGL_ = ys1;
        }else{
            xfollowWptGL_ = xs2;
            yfollowWptGL_ = ys2;
        }

        p1.x = xfollowWptGL_;
        p1.y = yfollowWptGL_;
        p1.z = Z_WPT*2;


        follow_waypoint_.pose.position.x = p1.x;
        follow_waypoint_.pose.position.y = p1.y;
        follow_waypoint_.pose.position.z = p1.z;

        followPoint_.xgl = xfollowWptGL_;
        followPoint_.ygl = yfollowWptGL_;

        double xtmp = xfollowWptGL_-xgl_;
        double ytmp = yfollowWptGL_-ygl_;

        followPoint_.xfs = cos_gl*xtmp + sin_gl*ytmp;
        followPoint_.yfs = -sin_gl*xtmp + cos_gl*ytmp;
        followPoint_.qfs_rad = atan2(followPoint_.yfs,followPoint_.xfs);

        followPoint_.qgl_rad = modAngle(yaw_ + followPoint_.qfs_rad);
        followPoint_.count = count;

        followPoint_.dist_front = distObject_;
        followPoint_.dist_follow = Dist_point_to_point(0,0,followPoint_.xfs,followPoint_.yfs);

        count ++;
        if(count > 1000) count = 0;

        pub_Followpoint_->publish(std::move(followPoint_));
    }

    lane_pp1_.x = xp2;
    lane_pp1_.y = yp2;
    lane_pp1_.z = Z_WPT;

    lane_pp2_.x = xp1;
    lane_pp2_.y = yp1;
    lane_pp2_.z = Z_WPT;

    lane_pp3_.x = xpp1;
    lane_pp3_.y = ypp1;
    lane_pp3_.z = Z_WPT;

    lane_pp4_.x = xpp2;
    lane_pp4_.y = ypp2;
    lane_pp4_.z = Z_WPT;

    return 1;
}



//点:(x1,y1),(x2,y2)を通り，点(x1,y1)から距離:w 離れた点(xp1,yp1),(xp2,yp2)
int Freespace_detector::Calc_rect4viz(double w, double x1,double y1,double x2,double y2,double *xp1, double *yp1, double *xpp1, double *ypp1){

    double ap=0;
    if(fabs(x2-x1) < 0.000000001){
        ap = 99999999999999;
    }else{
        //ap = -(x2-x1)/(y2-y1);
        ap = -(y2-y1)/(x2-x1);
    }

    *yp1 = -w/sqrt(ap*ap+1)+y1;
    *ypp1 = w/sqrt(ap*ap+1)+y1;

    *xp1 = ap*(*yp1-y1)+x1;
    *xpp1 = ap*(*ypp1-y1)+x1;

    return 1;
}

//点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
int Freespace_detector::Calc_CrossPoint4viz(double x1,double y1,double x2,double y2,double xp, double yp, double *xpp, double *ypp){

    double a=0;
    double ap=0;

    if(fabs(y2-y1) < 0.000000001){
        a = 99999999999999;
    }else{
        a = (x2-x1)/(y2-y1);
    }

    if(fabs(x2-x1) < 0.000000001){
        ap = 99999999999999;
    }else{
        ap = -(y2-y1)/(x2-x1);
    }

    *ypp = (a*y1-ap*yp-x1+xp)/(a-ap);
    *xpp = a*(*ypp-y1)+x1;

    return 1;
}


void Freespace_detector::Set_Feature_Waypoint(void){

    //#ifdef RVIZ
    //tf::TransformBroadcaster br;
    //tf::Transform transform;

   // Green
    if(editor_mode_ > 0){
        points_.color.r = 0.0f;
        points_.color.g = 156.0f/255.0f;
        points_.color.b = 209.0f/255.0f;
        points_.color.a = 1.0;
    }
    else{
        points_.color.g = 1.0f;
        //points_.color.r = 1.0f;
        points_.color.a = 1.0;
    }
    // Passed points
    if(editor_mode_ > 0){
        passed_points_.color.r = 0.8f;
        passed_points_.color.g = 0.8f;
        passed_points_.color.b = 0.0f;
        passed_points_.color.a = 1.0;
    }
    else{
        passed_points_.color.r = 0.6f;
        passed_points_.color.g = 0.6f;
        passed_points_.color.b = 0.6f;
        passed_points_.color.a = 1.0;
    }

    //Mark points
    if(editor_mode_ > 0){
        markpoints_.color.r = 0.5f;
        markpoints_.color.g = 0.8f;
        markpoints_.color.b = 0.9f;
        markpoints_.color.a = 1.0;
    }
    else{
        markpoints_.color.r = 0.5f;
        markpoints_.color.g = 0.8f;
        markpoints_.color.b = 0.9f;
        markpoints_.color.a = 1.0;
    }
}

void Freespace_detector::Init_Visualization(void){

    points_.points.clear();
    passed_points_.points.clear();
    markpoints_.points.clear();
    line_strip_.points.clear();
    current_waypoint_.points.clear();
    follow_waypoint_.points.clear();
    line_strip2_.points.clear();
    line_strip3_.points.clear();


    points_.id      = 0;
    line_strip_.id  = 1;
    cylinder_.id    = 2;
    current_waypoint_.id    = 3;
    wpt_string_.id          = 4;
    passed_points_.id       = 5;
    markpoints_.id          = 6;
    line_strip2_.id = 7;
    follow_waypoint_.id = 8;
    line_strip3_.id = 9;

    points_.type =  passed_points_.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    line_strip_.type     = visualization_msgs::msg::Marker::LINE_STRIP;
    cylinder_.type       = visualization_msgs::msg::Marker::CYLINDER;
    current_waypoint_.type       = visualization_msgs::msg::Marker::SPHERE;
    follow_waypoint_.type       = visualization_msgs::msg::Marker::SPHERE;
    wpt_string_.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markpoints_.type     = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip2_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip3_.type = visualization_msgs::msg::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points_.scale.x = passed_points_.scale.x = 0.3;
    points_.scale.y = passed_points_.scale.y = 0.3;

    //Waypoint scale
    current_waypoint_.scale.x = 0.4;
    current_waypoint_.scale.y = 0.4;
    current_waypoint_.scale.z = 0.4;

   //Follow Waypoint scale
    follow_waypoint_.scale.x = 0.4;
    follow_waypoint_.scale.y = 0.4;
    follow_waypoint_.scale.z = 0.4;

    // Waypoints:red
    current_waypoint_.color.r = 1.0f;
    current_waypoint_.color.g = 0.0f;
    current_waypoint_.color.b = 0.0f;
    current_waypoint_.color.a = 1.0;

    // Follow Waypoints: Blue
    follow_waypoint_.color.r = 0.25f;
    follow_waypoint_.color.g = 0.5f;
    follow_waypoint_.color.b = 1.0f;
    follow_waypoint_.color.a = 1.0;

    markpoints_.scale.x = 0.4;
    markpoints_.scale.y = 0.4;
    markpoints_.scale.z = 0.4;

    //cylinder
    cylinder_.scale.x = 1.0;
    cylinder_.scale.y = 1.0;
    cylinder_.scale.z = 0.01;

    cylinder_.color.r = 1.0;
    cylinder_.color.a = 0.4;

    //Waypoint string
    wpt_string_.scale.x = 1.2;
    wpt_string_.scale.y = 1.2;
    wpt_string_.scale.z = 1.2;
    wpt_string_.text = "-";

    //Waypoint color
    wpt_string_.color.r = 1.0f;
    wpt_string_.color.g = 1.0f;
    wpt_string_.color.b = 1.0f;
    wpt_string_.color.a = 1.0;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip_.scale.x = 0.04;
    line_strip2_.scale.x =  0.05;
    line_strip3_.scale.x =  0.05;

    // Line strip is white
    line_strip_.color.r = 0.8;
    line_strip_.color.g = 0.8;
    line_strip_.color.b = 0.8;
    line_strip_.color.a = 1.0;

    // Line list is red
    line_strip2_.color.r = 1.0;
    line_strip2_.color.a = 1.0;

    // Line list is blue
    line_strip3_.color.r = 0.25;
    line_strip3_.color.g = 0.55;
    line_strip3_.color.b = 1.0;
    line_strip3_.color.a = 1.0;

    //global座標系からみたrobot座標系の原点位置と方向の格納
    points_.header.frame_id = passed_points_.header.frame_id = line_strip_.header.frame_id = cylinder_.header.frame_id = current_waypoint_.header.frame_id = wpt_string_.header.frame_id = markpoints_.header.frame_id = "map"; //
    //line_strip2_.header.frame_id = "cloud_matching";
    follow_waypoint_.header.frame_id = "map";
    line_strip2_.header.frame_id = "map";
    line_strip3_.header.frame_id = "map";

    points_.header.stamp = passed_points_.header.stamp = line_strip_.header.stamp = cylinder_.header.stamp = current_waypoint_.header.stamp = wpt_string_.header.stamp = markpoints_.header.stamp = line_strip2_.header.stamp = line_strip3_.header.stamp = follow_waypoint_.header.stamp = this->get_clock()->now(); //
    points_.ns = passed_points_.ns =line_strip_.ns =  cylinder_.ns = "waypoint_array";
    current_waypoint_.ns = wpt_string_.ns  = "waypoint";
    follow_waypoint_.ns = "follow_waypoint";
    markpoints_.ns = "markpoint";
    line_strip2_.ns = "freelane";
    line_strip3_.ns = "egoregion";

    points_.action = passed_points_.action = line_strip_.action =  cylinder_.action =  current_waypoint_.action = wpt_string_.action = markpoints_.action = line_strip2_.action = line_strip3_.action = follow_waypoint_.action = visualization_msgs::msg::Marker::ADD; //
    points_.pose.orientation.w = passed_points_.pose.orientation.w = line_strip_.pose.orientation.w = cylinder_.pose.orientation.w =  current_waypoint_.pose.orientation.w = markpoints_.pose.orientation.w = line_strip2_.pose.orientation.w = follow_waypoint_.pose.orientation.w =1.0;//

    points_.pose.position.x = passed_points_.pose.position.x = markpoints_.pose.position.x = current_waypoint_.pose.position.x = line_strip2_.pose.position.x = line_strip3_.pose.position.x = follow_waypoint_.pose.position.x = 0;
    points_.pose.position.y = passed_points_.pose.position.y = markpoints_.pose.position.y = current_waypoint_.pose.position.y = line_strip2_.pose.position.y = line_strip3_.pose.position.y = follow_waypoint_.pose.position.y = 0;
    points_.pose.position.z = passed_points_.pose.position.z = markpoints_.pose.position.z = current_waypoint_.pose.position.z = line_strip2_.pose.position.z = line_strip3_.pose.position.z = follow_waypoint_.pose.position.z = Z_WPT;

}

int main(int argc, char * argv[])
{
  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // Waypiont_planner nodeの生成とSpin開始
  auto node = std::make_shared<Waypoint_planner>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
