#include "cvt_convert_functions.h"

#include <cmath>

namespace cvt{

    Matrix<double, 6, 1> toMat61XYZQuat(const vector<double>& targetPose){

        /*
            euler -> quaternion
            targetAngle    :: x, y, z, ex, ey, ez
            return retPose :: x, y, z, qx, qy, qz
        */

        Affine3d aff = Translation<double,3>(targetPose[0], targetPose[1], targetPose[2]) 
                    * AngleAxisd(targetPose[3], Vector3d::UnitX())
                    * AngleAxisd(targetPose[4], Vector3d::UnitY())
                    * AngleAxisd(targetPose[5], Vector3d::UnitZ());
        auto Trans = aff.translation();
        auto Q = Quaterniond(aff.rotation());
        //ES(Q.x()) ES(Q.y()) ES(Q.z()) EL(Q.w())
        Matrix<double, 6,1> retPose;
        for(int i=0;i<3;i++) retPose(i,0) = Trans(i);
        retPose(3,0) = Q.x();
        retPose(4,0) = Q.y();
        retPose(5,0) = Q.z();
        //EL(retPose)
        return  retPose;
    }

    Matrix<double, 6, 1> toMat61XYZQuat(const Matrix4d& tipPose){
        Quaterniond Q(Affine3d(tipPose).rotation());
        Matrix<double, 6, 1> retX;
        for(int i=0;i<3;i++) retX(i,0) = tipPose(i,3);
        retX(3,0) = Q.x();
        retX(4,0) = Q.y();
        retX(5,0) = Q.z();
        return retX;
    }

    Matrix<double, 6, 1> toMat61XYZEuler(const vector<double>& targetPose){
        Affine3d aff = Translation<double,3>(targetPose[0], targetPose[1], targetPose[2]) 
                    * AngleAxisd(targetPose[3], Vector3d::UnitX())
                    * AngleAxisd(targetPose[4], Vector3d::UnitY())
                    * AngleAxisd(targetPose[5], Vector3d::UnitZ());
        auto Trans = aff.translation();
        auto Euler = aff.rotation().eulerAngles(0,1,2);
        Matrix<double, 6,1> retPose;
        for(int i=0;i<3;i++) retPose(i,0) = Trans(i);
        for(int i=0;i<3;i++) retPose(i+3,0) = Euler(i);
        return  retPose;
    }

    Matrix<double, 6, 1> toMat61XYZEuler(const Matrix4d& tipPose){
        auto Euler = Affine3d(tipPose).rotation().eulerAngles(0,1,2);
        Matrix<double, 6, 1> retX;
        for(int i=0;i<3;i++) retX(i,0) = tipPose(i,3);
        for(int i=0;i<3;i++) retX(i+3,0) = Euler(i);
        return retX;
    }


    vector<double> toVecXYZEuler(const Matrix4d& Mat){
        Affine3d aff(Mat);
        Vector3d trans = aff.translation();
        Eigen::Vector3d euler = aff.rotation().eulerAngles(0,1,2);
        vector<double> retV(6);
        for(int i=0;i<3;i++) retV[i] = trans(i);
        for(int i=0;i<3;i++) retV[i+3] = euler(i);
        return retV;
    }


    Matrix4d toMat44RotZ(const double& angle){
        //TO DO :: reduce calculation
        Affine3d aff = Translation<double,3>(0.,0.,0.) * AngleAxisd(angle, Vector3d::UnitZ());
        Matrix4d retT;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) retT(i,j) = aff(i,j);
        return retT;
    }


    Matrix4d toMat44FromDH(const DHParam& dh){
        //ref http:  kuwamai hatenablog 211232
        //arg: dJ->jointAngle        
        double a = dh.a;
        double alp = dh.alp;
        double d = dh.d;
        double tht = dh.tht;
        double ca = cos(alp), sa = sin(alp);
        double ct = cos(tht), st = sin(tht);

        Matrix4d retT;
        retT<<
            ct,    -st,    0.,     a,
            ca*st, ca*ct, -sa, -sa*d,
            sa*st, sa*ct,  ca,  ca*d,
            0.,     0.,     0.,     1.;
        
        return retT;
    }

    Matrix4d toMat44TRFromDH(const DHParam& dh, const double& jointAngle){
        double a = dh.a;
        double alp = dh.alp;
        double d = dh.d;
        double tht = dh.tht;
        tht += jointAngle;
        double ca = cos(alp), sa = sin(alp);
        double ct = cos(tht), st = sin(tht);

        Matrix4d retT;
        retT<<
            ct,    -st,    0.,     a,
            ca*st, ca*ct, -sa, -sa*d,
            sa*st, sa*ct,  ca,  ca*d,
            0.,     0.,     0.,     1.;
        
        return retT;
    }

    Matrix4d toMat44RTFromDH(const double& jointAngle, const DHParam& dh){
        double a = dh.a;
        double alp = dh.alp;
        double d = dh.d;
        double tht = dh.tht;
        double ca = cos(alp), sa = sin(alp);
        double ct = cos(tht), st = sin(tht);
        double ja = jointAngle;
        double cj = cos(ja), sj = sin(ja);

        Matrix4d retT;
        retT<<
            cj*ct-sj*ca*st, -cj*st-sj*ca*ct, sj*sa,  a*cj+d*sj*sa,
            sj*ct+cj*ca*st, -sj*st+cj*ca*ct, -cj*sa, a*sj-d*cj*sa,
            sa*st,          sa*ct,           ca,     d*ca,
            0.,             0.,              0.,      1.;
        
        return retT;
    }

    vector<double> fromTouchX2Cobotta(const vector<double> vec){
        Eigen::Quaterniond quat(vec[3], vec[4], vec[5], vec[6]);
        Eigen::Translation<double,3> trans(vec[0], vec[1], vec[2]);
        Eigen::Affine3d aff = trans*quat*AngleAxisd(M_PI/2., Vector3d::UnitX());
        vector<double> ret(6);
        //position
        for(int i=0;i<3;i++) ret[i] = aff(i,3);
        //orientation
        auto euler = aff.rotation().eulerAngles(0,1,2);
        for(int i=0;i<3;i++) ret[i+3] = euler(i);
        return ret;
    }



}