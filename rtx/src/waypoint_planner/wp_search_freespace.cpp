#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "waypoint_planner_node.hpp"

//#define NOIZE_POINTS 3
//#define NOIZE_POINTS 1 //251204
#define NOIZE_POINTS 10 //251204

int Waypoint_planner::Search_freespace(double dist_range, double qdeg_range, double free_width,double dqdeg){

    deque <PositionBase> dq_point;

    double xGL = xGL_;
    double yGL = yGL_;
    double qradGL = yawGL_;

    double dist = dist_range;
    double line_dist = 0;

    double qdegA_left_FS=0;
    double qradA_left_FS=0;
    double distA_left_FS=dist;

    double dist_urg3d_edgeGL=0;

    double qdegA_right_FS=0;
    double qradA_right_FS=0;
    double distA_right_FS=dist;

    double qdegA_FS=0;
    double qradA_FS=0;

    double qrad0_FS=0;
    double qdeg0_FS=0;

    double qrad_search_start=0;
    double qrad_search_end=0;


    double distA_FS=dist;
    double dist_urg_point = 0;

    double eval = 0;
    double max_eval = 0;

    double min_eval_left = 9999;
    double min_eval_right = 9999;

    int ret_valL = 0;
    int ret_valR = 0;

    static PositionBase l_tmppointFS;
    static PositionBase l_tmppointGL;

    static PositionBase tmpPointFS;
    static PositionBase tmpPointGL;

    static PositionBase dstpointFS;

    xy tWpt;
    xy WptFS;
    
    xy nearestPt={9999,9999};
    xy nearestPt_L={9999,9999};
    xy nearestPt_R={9999,-9999};
    
    
    static xy tmp_WptGL;
    static xy tmp_WptFS;

    static bool flag_init = true;
    static unsigned int count=0;
  
    double cos_qGL = cos(qradGL);
    double sin_qGL = sin(qradGL);

    point3D ptObj;

    tWpt.x = current_wpt_.xgl - xGL;
    tWpt.y = current_wpt_.ygl - yGL;

    tmp_WptFS.x = WptFS.x = (cos_qGL*tWpt.x + sin_qGL*tWpt.y);
    tmp_WptFS.y = WptFS.y = (-sin_qGL*tWpt.x + cos_qGL*tWpt.y);

    //qrad0_FS = qradA_FS = atan2(tmp_WptFS.y,tmp_WptFS.x);
    qrad0_FS = qradA_FS = atan2(tmp_WptFS.y,tmp_WptFS.x);
    qdeg0_FS = qdegA_FS = qrad0_FS*M_RAD2DEG;

    //printf("WP::Search_freespace()\n");

    int countL=0;
    int countR=0;

    double minDistLeft = 9999;
    double minDistRight = 9999;
    double nearestPt_dist = 9999;
    double last_free_width = free_width;

    double tmp_xGL = xGL_;
    double tmp_yGL = yGL_;
    
    double tmp_xFS=0;
    double tmp_yFS=0;

    int side = -1;

    double R = -1;
    double r = 9999;

    double tmp_free_width = free_width;

    const double dist_noize_cut = 0.35; //!!!　重要！！

    int min_count_left = 9999;
    int min_count_right = 9999;

    for(double tmp_free_width = 1.0*free_width; tmp_free_width >= free_width; tmp_free_width -=0.05 ){ //無効化

        //LEFT side
        qrad_search_start = qdeg0_FS*M_DEG2RAD;
        qrad_search_end= (qdeg0_FS + qdeg_range)*M_DEG2RAD;
        countL=-1;

        minDistLeft = 9999;
        minDistRight = 9999;
        
        if(current_wpt_.yfs >= 0){ //Left
        
            //Min dist
            for(double qdeg_FS=qdeg0_FS; qdeg_FS < qdeg0_FS + qdeg_range; qdeg_FS+= dqdeg){

                count = 0;
                eval = 0;
                //2DURG
                minDistLeft = dist_range;
    
        #if 1     
                for(uint k=0;k < vec_Object_.size();k++ ){
        
                    ptObj = vec_Object_[k];        
                    dist = Dist_point_to_point(0,0,ptObj.x,ptObj.y);

                    line_dist = Dist_from_point_to_line(ptObj.x,ptObj.y,0,0,qdeg_FS);
                    //printf("FD:Left: line_dist = %lf,free_width =%lf \n",line_dist ,free_width);
                    if(line_dist < tmp_free_width && dist > dist_noize_cut && dist < dist_range){ //!!! noize
                        count++;
                        eval = count;
                        if(dist < minDistLeft){
                            minDistLeft = dist;
                            nearestPt_L.x = ptObj.x;
                            nearestPt_L.y = ptObj.y;
                        }
                    }
                }

                //printf("WP:Left: current_wpt_.yfs = %lf, qdeg_FS = %lf,minDistLeft = %lf, count =%d,tmp_free_width=%lf \n",
                //    current_wpt_.yfs, qdeg_FS,minDistLeft, count,tmp_free_width);

        #endif         
                if(count < min_count_left){ //Find FreeSpace

                    trans_DistAngle_to_FS(minDistLeft, qdeg_FS*M_DEG2RAD, &tmp_xFS, &tmp_yFS);
                    trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);

                    if(area_map_.valueWorld(tmp_xGL,tmp_yGL) == 0){           
                       
                        qdegA_left_FS= qdeg_FS;
                        distA_left_FS = minDistLeft;
                        ret_valL = 1;
                    
                        //printf("WP:Left freespace: clear :: minDistLeft=%lf, qdeg_FS=%lf, xFS=%lf, yFS=%lf ---\n",minDistLeft,qdeg_FS,tmp_xFS,tmp_yFS);
                        
                        countL = count;     
                        min_count_left = count;
                    }
                }
            }

            nearestPt = nearestPt_L;
            qdegA_FS = qdegA_left_FS;
            distA_FS= distA_left_FS;
        }


        // printf("lane_side =%d\n",lane_side_);
        // printf("L:countL =%d,min_eval_left=%lf\n",countL,min_eval_left);
        // printf("L:ret_valL =%d\n",ret_valL );
        
        //printf("FD: distA_left_FS=%lf, qdegA_left_FS=%lf, vec_Object_.size()=%d\n",distA_left_FS, qdegA_left_FS,vec_Object_.size());        
        else{  //Right Side
            qrad_search_start = qdeg0_FS*M_DEG2RAD;
            qrad_search_end= (qdeg0_FS - qdeg_range)*M_DEG2RAD;
            countR=-1;

            for(double qdeg_FS=qdeg0_FS; qdeg_FS > qdeg0_FS - qdeg_range/*30*/; qdeg_FS-= dqdeg){
                count = 0;
                eval = 0;
 
                minDistRight = dist_range;
#if 1        
                for(uint k=0;k < vec_Object_.size();k++ ){
                
                    ptObj = vec_Object_[k];
                    dist = Dist_point_to_point(0,0,ptObj.x,ptObj.y);
                
                    line_dist = Dist_from_point_to_line(ptObj.x,ptObj.y,0,0,qdeg_FS);
                    //printf("FD:Right: line_dist = %lf \n",line_dist );

                    if(line_dist < tmp_free_width && dist > dist_noize_cut && dist < dist_range){
                        count++;
                        eval = count;
                    
                        if(dist < minDistRight){
                            minDistRight = dist;
                            nearestPt_R.x = ptObj.x;
                            nearestPt_R.y = ptObj.y;
                        }
                    }
                }
#endif            
                //printf("WP:Right: current_wpt_.yfs = %lf, qdeg_FS = %lf,minDistRight = %lf, count =%d,tmp_free_width=%lf \n"
                //    ,current_wpt_.yfs,qdeg_FS,minDistRight, count,tmp_free_width);

                if(count < min_count_right){ //Find FreeSpace

                    trans_DistAngle_to_FS(minDistRight, qdeg_FS*M_DEG2RAD, &tmp_xFS, &tmp_yFS);
                    trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);

                    if(area_map_.valueWorld(tmp_xGL,tmp_yGL) == 0){           
                       
                        qdegA_right_FS= qdeg_FS;
                        distA_right_FS = minDistRight;
                        ret_valR = 1;
                    
                        //printf("WP:Right freespace: clear :: minDistRiht=%lf, qdeg_FS=%lf, xFS=%lf, yFS=%lf ---\n",
                        //    minDistRight,qdeg_FS,tmp_xFS,tmp_yFS);
                        
                        countR = count;     
                        min_count_right = count;
                    }
                }  
            }

            nearestPt = nearestPt_R;
            qdegA_FS = qdegA_right_FS;
            distA_FS= distA_right_FS;
        }
        //printf("FD: distA_right_FS=%lf,  qdegA_right_FS=%lf,vec_Object_.size()=%d\n\n",distA_right_FS, qdegA_right_FS,vec_Object_.size());

        if(flag_init == false){
            l_tmppointFS = tmpPointFS;
            l_tmppointGL = tmpPointGL;
        }
    }

    nearestPt_dist = Dist_point_to_point(0,0,nearestPt.x,nearestPt.y);

    #if 0
    printf("dist_range=%2.1lf,distA_left_FS=%2.1lf,distA_right_FS=%2.1lf\n",dist_range,distA_left_FS,distA_right_FS);
    printf("last_free_width=%lf / %lf\n",last_free_width,free_width);
    printf("qdegA_FS=%lf\n",qdegA_FS);
    printf("countL=%d,countR=%d\n",countL,countR);
    printf("ret_valL=%d,ret_valR=%d\n\n",ret_valL,ret_valR);
    #endif

    //printf("FD: distA_FS=%lf \n",distA_FS);
    //旋回半径
    //xy targetPt = {9999,-9999};
        
    tmp_xFS=0;tmp_yFS=0;
    double tmp_qradFS=0;
    tmpPointFS.dist = distA_FS;
    tmpPointFS.qrad = qdegA_FS*M_DEG2RAD;

    tmp_xGL = xGL_;
    tmp_yGL = yGL_;

    trans_DistAngle_to_FS(dist_range, tmpPointFS.qrad, &tmp_xFS, &tmp_yFS);
    trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);
    
    tmpPointGL.x = tmp_xGL;
    tmpPointGL.y = tmp_yGL;
    
    trans_GL_to_FS(tmpPointGL.x, tmpPointGL.y, &tmp_xFS, &tmp_yFS, &tmp_qradFS);
    
    tmp_followPoint_.xfs = tmp_xFS;
    tmp_followPoint_.yfs = tmp_yFS;
    tmp_followPoint_.qfs_rad = tmp_qradFS;
    
    tmp_followPoint_.qgl_rad = modAngle(yawGL_ + tmp_followPoint_.qfs_rad);
    tmp_followPoint_.count = count;    
    tmp_followPoint_.dist_follow = Dist_point_to_point(0,0,tmp_followPoint_.xfs,tmp_followPoint_.yfs);


    static geometry_msgs::msg::Point p1;

    p1.x = tmpPointGL.x;
    p1.y = tmpPointGL.y;
    p1.z = Z_WPT*2;
    

    //狭い区間の通り抜け
    if(minLeftDist_ > 0.7|| minRightDist_ > 0.7){
        follow_waypoint_.pose.position.x = p1.x;
        follow_waypoint_.pose.position.y = p1.y;
        follow_waypoint_.pose.position.z = p1.z;
    }
    if(count++ > 1000) count = 0;

    flag_init = false;

    return 1;
}

int Waypoint_planner::Search_freespace2(double dist_range, double qdeg_range_left,double qdeg_range_right, double free_width,double dqdeg){

    deque <PositionBase> dq_point;

    double xGL = xGL_;
    double yGL = yGL_;
    double qradGL = yawGL_;

    double dist = dist_range;
    double line_dist = 0;

    double qdegA_left_FS=0;
    double qradA_left_FS=0;
    double distA_left_FS=dist;

    double dist_urg3d_edgeGL=0;

    double qdegA_right_FS=0;
    double qradA_right_FS=0;
    double distA_right_FS=dist;

    double qdegA_FS=0;
    double qradA_FS=0;

    double qrad0_FS=0;
    double qdeg0_FS=0;

    double qrad_search_start=0;
    double qrad_search_end=0;


    double distA_FS=dist;
    double dist_urg_point = 0;

    static PositionBase tmpPointFS;
    static PositionBase tmpPointGL;

    static PositionBase dstpointFS;

    xy tWpt;
    xy WptFS;
        
    static xy tmp_WptGL;
    static xy tmp_WptFS;

    static unsigned int count=0;
  
    double cos_qGL = cos(qradGL);
    double sin_qGL = sin(qradGL);

    point3D ptObj;

    tWpt.x = current_wpt_.xgl - xGL;
    tWpt.y = current_wpt_.ygl - yGL;

    tmp_WptFS.x = WptFS.x = (cos_qGL*tWpt.x + sin_qGL*tWpt.y);
    tmp_WptFS.y = WptFS.y = (-sin_qGL*tWpt.x + cos_qGL*tWpt.y);

    qrad0_FS = qradA_FS = atan2(tmp_WptFS.y,tmp_WptFS.x);
    qdeg0_FS = qdegA_FS = qrad0_FS*M_RAD2DEG;

    //printf("WP::Search_freespace()\n");

    int min_countL=9999;
    int min_countR=9999;

    double minDistLeft = 9999;
    double minDistRight = 9999;
    double last_free_width = free_width;

    double tmp_xGL = xGL_;
    double tmp_yGL = yGL_;
    
    double tmp_xFS=0;
    double tmp_yFS=0;

    double tmp_free_width = free_width;

    const double dist_noize_cut = 0.35; //!!!　重要！！
    
    double eval_L = 9999;
    double eval_R = 9999;

    for(double tmp_free_width = 1.0*free_width; tmp_free_width >= free_width; tmp_free_width -=0.05 ){ //無効化

        //LEFT side       
        minDistLeft = 9999;
        minDistRight = 9999;
        
        /// Left            
        for(double qdeg_FS=qdeg0_FS; qdeg_FS < qdeg0_FS + qdeg_range_left; qdeg_FS+= dqdeg){

            count = 0;
            
            //2DURG
            minDistLeft = dist_range;
        
            for(uint k=0;k < vec_Object_.size();k++ ){
        
                ptObj = vec_Object_[k];        
                dist = Dist_point_to_point(0,0,ptObj.x,ptObj.y);

                line_dist = Dist_from_point_to_line(ptObj.x,ptObj.y,0,0,qdeg_FS);
                //printf("FD:Left: line_dist = %lf,free_width =%lf \n",line_dist ,free_width);
                if(line_dist < tmp_free_width && dist > dist_noize_cut && dist < dist_range){ //!!! noize
                    count++;
                    
                    if(dist < minDistLeft){
                        minDistLeft = dist;
                    }
                }
            }

            //printf("WP:Left: current_wpt_.yfs = %lf, qdeg_FS = %lf,minDistLeft = %lf, count =%d,tmp_free_width=%lf \n",
            //        current_wpt_.yfs, qdeg_FS,minDistLeft, count,tmp_free_width);

            if(count < min_countL){ //Find FreeSpace

                trans_DistAngle_to_FS(minDistLeft, qdeg_FS*M_DEG2RAD, &tmp_xFS, &tmp_yFS);
                trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);

                if(area_map_.valueWorld(tmp_xGL,tmp_yGL) == 0){           
                       
                    qdegA_left_FS= qdeg_FS;
                    distA_left_FS = minDistLeft;
                    
                    //printf("WP:Left freespace: clear :: minDistLeft=%lf, qdeg_FS=%lf, xFS=%lf, yFS=%lf ---\n",minDistLeft,qdeg_FS,tmp_xFS,tmp_yFS);
                    min_countL = count;             
                }
            }
        }

        // printf("lane_side =%d\n",lane_side_);
        // printf("L:countL =%d,min_eval_left=%lf\n",countL,min_eval_left);
        // printf("L:ret_valL =%d\n",ret_valL );
        
        //printf("FD: distA_left_FS=%lf, qdegA_left_FS=%lf, vec_Object_.size()=%d\n",distA_left_FS, qdegA_left_FS,vec_Object_.size());

        ///Right Side    
        for(double qdeg_FS=qdeg0_FS; qdeg_FS > qdeg0_FS - qdeg_range_right; qdeg_FS-= dqdeg){
            count = 0;
            minDistRight = dist_range;

            for(uint k=0;k < vec_Object_.size();k++ ){
                
                ptObj = vec_Object_[k];
                dist = Dist_point_to_point(0,0,ptObj.x,ptObj.y);
                
                line_dist = Dist_from_point_to_line(ptObj.x,ptObj.y,0,0,qdeg_FS);
                //printf("FD:Right: line_dist = %lf \n",line_dist );

                if(line_dist < tmp_free_width && dist > dist_noize_cut && dist < dist_range){
                    count++;
                    
                    if(dist < minDistRight){
                        minDistRight = dist;
                    }
                }
            }
        
            //printf("WP:Right: current_wpt_.yfs = %lf, qdeg_FS = %lf,minDistRight = %lf, count =%d,tmp_free_width=%lf \n"
            //    ,current_wpt_.yfs,qdeg_FS,minDistRight, count,tmp_free_width);

            if(count < min_countR){ //Find FreeSpace

                trans_DistAngle_to_FS(minDistRight, qdeg_FS*M_DEG2RAD, &tmp_xFS, &tmp_yFS);
                trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);

                if(area_map_.valueWorld(tmp_xGL,tmp_yGL) == 0){           
                       
                    qdegA_right_FS= qdeg_FS;
                    distA_right_FS = minDistRight;
                    
                    //printf("WP:Right freespace: clear :: minDistRiht=%lf, qdeg_FS=%lf, xFS=%lf, yFS=%lf ---\n",
                    //    minDistRight,qdeg_FS,tmp_xFS,tmp_yFS);
                        
                    min_countR = count;                        
                }
            }  
        }

        eval_L = min_countL*minDistLeft;
        eval_R = min_countR*minDistRight;

        //
        if(min_countL < min_countR){
            qdegA_FS = qdegA_left_FS;
            distA_FS= distA_left_FS;
        }
        else if(min_countR < 9999){
            qdegA_FS = qdegA_right_FS;
            distA_FS= distA_right_FS;
        }
        else{
            qdegA_FS = 0;
            distA_FS= distWpt_;
        }
        //printf("FD: distA_right_FS=%lf,  qdegA_right_FS=%lf,vec_Object_.size()=%d\n\n",distA_right_FS, qdegA_right_FS,vec_Object_.size());

    }

    #if 0
    printf("dist_range=%2.1lf,distA_left_FS=%2.1lf,distA_right_FS=%2.1lf\n",dist_range,distA_left_FS,distA_right_FS);
    printf("last_free_width=%lf / %lf\n",last_free_width,free_width);
    printf("qdegA_FS=%lf\n",qdegA_FS);
    printf("countL=%d,countR=%d\n",countL,countR);
    printf("ret_valL=%d,ret_valR=%d\n\n",ret_valL,ret_valR);
    #endif
    
    tmp_xFS=0;tmp_yFS=0;
    double tmp_qradFS=0;
    tmpPointFS.dist = distA_FS;
    tmpPointFS.qrad = qdegA_FS*M_DEG2RAD;

    tmp_xGL = xGL_;
    tmp_yGL = yGL_;

    trans_DistAngle_to_FS(dist_range, tmpPointFS.qrad, &tmp_xFS, &tmp_yFS);
    trans_FS_to_GL(tmp_xFS, tmp_yFS, &tmp_xGL, &tmp_yGL);
    
    tmpPointGL.x = tmp_xGL;
    tmpPointGL.y = tmp_yGL;
    
    trans_GL_to_FS(tmpPointGL.x, tmpPointGL.y, &tmp_xFS, &tmp_yFS, &tmp_qradFS);
    
    tmp_followPoint_.xfs = tmp_xFS;
    tmp_followPoint_.yfs = tmp_yFS;
    tmp_followPoint_.qfs_rad = tmp_qradFS;
    
    tmp_followPoint_.qgl_rad = modAngle(yawGL_ + tmp_followPoint_.qfs_rad);
    tmp_followPoint_.count = count;    
    tmp_followPoint_.dist_follow = Dist_point_to_point(0,0,tmp_followPoint_.xfs,tmp_followPoint_.yfs);


    static geometry_msgs::msg::Point p1;

    p1.x = tmpPointGL.x;
    p1.y = tmpPointGL.y;
    p1.z = Z_WPT*2;
    
    follow_waypoint_.pose.position.x = p1.x;
    follow_waypoint_.pose.position.y = p1.y;
    follow_waypoint_.pose.position.z = p1.z;

    return 1;
}

int Waypoint_planner::trans_DistAngle_to_FS(double dist, double qrad_FS, double *xFS, double *yFS){

    *xFS = dist*cos(qrad_FS);
    *yFS = dist*sin(qrad_FS);

    return 1;
}


int Waypoint_planner::trans_FS_to_GL(double xFS, double yFS, double *xGL, double *yGL){

    double cos_qGL = cos(yawGL_);
    double sin_qGL = sin(yawGL_);

    double tmp_xGL = (cos_qGL*xFS - sin_qGL*yFS);
    double tmp_yGL = (sin_qGL*xFS + cos_qGL*yFS);

    *xGL = tmp_xGL + xGL_;
    *yGL = tmp_yGL + yGL_;

    return 1;
}

int Waypoint_planner::trans_FS_to_GL2(double dist_range, double qrad_FS, double *xGL, double *yGL){
    
    double xFS = dist_range*cos(qrad_FS);
    double yFS = dist_range*sin(qrad_FS);

    double cos_qGL = cos(yawGL_);
    double sin_qGL = sin(yawGL_);
    
    double tmp_xGL = (cos_qGL*xFS - sin_qGL*yFS);
    double tmp_yGL = (sin_qGL*xFS + cos_qGL*yFS);

    *xGL = tmp_xGL + xGL_;
    *yGL = tmp_yGL + yGL_;

    return 1;
}

int Waypoint_planner::trans_GL_to_FS(double xGL, double yGL, double *xFS, double *yFS,double *qrad_FS){

    double tmp_xFS = xGL - xGL_;
    double tmp_yFS = yGL - yGL_;

    double cos_GL = cos(yawGL_);
    double sin_GL = sin(yawGL_);

    *xFS = cos_GL*tmp_xFS + sin_GL*tmp_yFS;
    *yFS = -sin_GL*tmp_xFS + cos_GL*tmp_yFS;
    *qrad_FS = atan2(*yFS,*xFS);
    
    return 1;
}

int Waypoint_planner::trans_GL_to_FSf(double xGL, double yGL, float *xFS, float *yFS,float *qrad_FS){

    double tmp_xFS = xGL - xGL_;
    double tmp_yFS = yGL - yGL_;

    double cos_GL = cos(yawGL_);
    double sin_GL = sin(yawGL_);

    *xFS = cos_GL*tmp_xFS + sin_GL*tmp_yFS;
    *yFS = -sin_GL*tmp_xFS + cos_GL*tmp_yFS;
    *qrad_FS = atan2(*yFS,*xFS);
    
    return 1;
}

