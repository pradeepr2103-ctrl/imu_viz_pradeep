#pragma once
#include <cstring>
#include <cmath>

namespace mocap {

struct Vec3 { float x=0, y=0, z=0; };
struct Vec4 { float x=0, y=0, z=0, w=1; };
struct Quat { float w=1, x=0, y=0, z=0; };

struct Mat4 {
    float m[16];  // column-major

    Mat4() { std::memset(m,0,sizeof(m)); m[0]=m[5]=m[10]=m[15]=1.f; }

    Mat4 operator*(const Mat4& b) const {
        Mat4 r; std::memset(r.m,0,sizeof(r.m));
        for(int c=0;c<4;c++)
            for(int row=0;row<4;row++)
                for(int k=0;k<4;k++)
                    r.m[c*4+row] += m[k*4+row]*b.m[c*4+k];
        return r;
    }

    static Mat4 identity() { return Mat4(); }
    static Mat4 fromQuat(const Quat& q);
    static Mat4 fromTRS(const Vec3& t, const Quat& r, const Vec3& s);
    static Mat4 translation(float tx, float ty, float tz);
    static Mat4 scale(float sx, float sy, float sz);
    static Mat4 perspective(float fovY, float aspect, float zn, float zf);
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
};

inline Mat4 Mat4::fromQuat(const Quat& q) {
    float x=q.x, y=q.y, z=q.z, w=q.w;
    // normalize
    float len = sqrtf(w*w+x*x+y*y+z*z);
    if(len > 0.0001f){ w/=len; x/=len; y/=len; z/=len; }
    Mat4 r;
    r.m[0] =1-2*(y*y+z*z); r.m[4]=2*(x*y-w*z);   r.m[8] =2*(x*z+w*y);  r.m[12]=0;
    r.m[1] =2*(x*y+w*z);   r.m[5]=1-2*(x*x+z*z); r.m[9] =2*(y*z-w*x);  r.m[13]=0;
    r.m[2] =2*(x*z-w*y);   r.m[6]=2*(y*z+w*x);   r.m[10]=1-2*(x*x+y*y);r.m[14]=0;
    r.m[3] =0;              r.m[7]=0;              r.m[11]=0;             r.m[15]=1;
    return r;
}

inline Mat4 Mat4::fromTRS(const Vec3& t, const Quat& r, const Vec3& s) {
    Mat4 R = fromQuat(r);
    Mat4 result;
    // scale columns then set translation
    for(int row=0;row<3;row++){
        result.m[0*4+row] = R.m[0*4+row]*s.x;
        result.m[1*4+row] = R.m[1*4+row]*s.y;
        result.m[2*4+row] = R.m[2*4+row]*s.z;
    }
    result.m[12]=t.x; result.m[13]=t.y; result.m[14]=t.z; result.m[15]=1;
    return result;
}

inline Mat4 Mat4::translation(float tx,float ty,float tz) {
    Mat4 r; r.m[12]=tx; r.m[13]=ty; r.m[14]=tz; return r;
}

inline Mat4 Mat4::scale(float sx,float sy,float sz) {
    Mat4 r; r.m[0]=sx; r.m[5]=sy; r.m[10]=sz; return r;
}

inline Mat4 Mat4::perspective(float fovY,float aspect,float zn,float zf) {
    Mat4 r; std::memset(r.m,0,sizeof(r.m));
    float f=1.f/tanf(fovY*0.5f);
    r.m[0]=f/aspect; r.m[5]=f;
    r.m[10]=(zf+zn)/(zn-zf); r.m[11]=-1.f;
    r.m[14]=(2.f*zf*zn)/(zn-zf);
    return r;
}

inline Mat4 Mat4::lookAt(const Vec3& eye,const Vec3& target,const Vec3& up) {
    Vec3 f={target.x-eye.x,target.y-eye.y,target.z-eye.z};
    float fl=sqrtf(f.x*f.x+f.y*f.y+f.z*f.z); if(fl>0){f.x/=fl;f.y/=fl;f.z/=fl;}
    Vec3 s={f.y*up.z-f.z*up.y, f.z*up.x-f.x*up.z, f.x*up.y-f.y*up.x};
    float sl=sqrtf(s.x*s.x+s.y*s.y+s.z*s.z); if(sl>0){s.x/=sl;s.y/=sl;s.z/=sl;}
    Vec3 u={s.y*f.z-s.z*f.y, s.z*f.x-s.x*f.z, s.x*f.y-s.y*f.x};
    Mat4 r;
    r.m[0]=s.x; r.m[4]=s.y; r.m[8] =s.z;  r.m[12]=-(s.x*eye.x+s.y*eye.y+s.z*eye.z);
    r.m[1]=u.x; r.m[5]=u.y; r.m[9] =u.z;  r.m[13]=-(u.x*eye.x+u.y*eye.y+u.z*eye.z);
    r.m[2]=-f.x;r.m[6]=-f.y;r.m[10]=-f.z; r.m[14]= (f.x*eye.x+f.y*eye.y+f.z*eye.z);
    r.m[15]=1.f;
    return r;
}

} // namespace mocap
