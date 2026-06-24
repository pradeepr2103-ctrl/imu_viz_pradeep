#pragma once
#include <cstring>
#include <cmath>

namespace mocap {

struct Vec3 { float x=0, y=0, z=0; };
struct Vec4 { float x=0, y=0, z=0, w=1; };
struct Quat { float w=1, x=0, y=0, z=0; };

// Quaternion helpers
inline Quat quatMul(const Quat& a, const Quat& b){
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

inline Quat quatInverse(const Quat& q){
    float n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if(n < 1e-8f) return {1,0,0,0};
    return {q.w/n, -q.x/n, -q.y/n, -q.z/n};
}

inline Quat quatNorm(const Quat& q){
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if(n < 1e-8f) return {1,0,0,0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

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

    // Extract rotation-only quaternion from column-major TRS matrix
    Quat toQuat() const {
        // assumes uniform scale — extract from upper-left 3x3
        // columns of m: X-axis=m[0..2], Y-axis=m[4..6], Z-axis=m[8..10]
        float sx = sqrtf(m[0]*m[0]+m[1]*m[1]+m[2]*m[2]);
        float sy = sqrtf(m[4]*m[4]+m[5]*m[5]+m[6]*m[6]);
        float sz = sqrtf(m[8]*m[8]+m[9]*m[9]+m[10]*m[10]);
        if(sx<1e-8f) sx=1; if(sy<1e-8f) sy=1; if(sz<1e-8f) sz=1;
        // normalized rotation matrix elements
        float r00=m[0]/sx, r10=m[1]/sx, r20=m[2]/sx;
        float r01=m[4]/sy, r11=m[5]/sy, r21=m[6]/sy;
        float r02=m[8]/sz, r12=m[9]/sz, r22=m[10]/sz;
        float tr = r00+r11+r22;
        Quat q;
        if(tr > 0){
            float s=0.5f/sqrtf(tr+1.0f);
            q.w=0.25f/s; q.x=(r21-r12)*s; q.y=(r02-r20)*s; q.z=(r10-r01)*s;
        } else if(r00>r11 && r00>r22){
            float s=2.0f*sqrtf(1.0f+r00-r11-r22);
            q.w=(r21-r12)/s; q.x=0.25f*s; q.y=(r01+r10)/s; q.z=(r02+r20)/s;
        } else if(r11>r22){
            float s=2.0f*sqrtf(1.0f+r11-r00-r22);
            q.w=(r02-r20)/s; q.x=(r01+r10)/s; q.y=0.25f*s; q.z=(r12+r21)/s;
        } else {
            float s=2.0f*sqrtf(1.0f+r22-r00-r11);
            q.w=(r10-r01)/s; q.x=(r02+r20)/s; q.y=(r12+r21)/s; q.z=0.25f*s;
        }
        return quatNorm(q);
    }

    // Extract translation
    Vec3 getTranslation() const { return {m[12], m[13], m[14]}; }

    // Scale of each column
    Vec3 getScale() const {
        return {
            sqrtf(m[0]*m[0]+m[1]*m[1]+m[2]*m[2]),
            sqrtf(m[4]*m[4]+m[5]*m[5]+m[6]*m[6]),
            sqrtf(m[8]*m[8]+m[9]*m[9]+m[10]*m[10])
        };
    }
};

inline Mat4 Mat4::fromQuat(const Quat& q) {
    float x=q.x, y=q.y, z=q.z, w=q.w;
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
