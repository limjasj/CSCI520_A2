#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <cmath>
#include "vector.h"
#include <iostream>

bool isGraphOne = false;
bool isGraphTwo = false;
bool isGraphThree = false;
bool isGraphFour = false;

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion* pInputMotion, Motion** pOutputMotion, int N)
{
    double euler[3] = { -90,-70,30 };
    Quaternion<double> q;
    double out[3];

    Euler2Quaternion(euler, q);
    Quaternion2Euler(q, out);

    //Allocate new motion
    *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

    //Perform the interpolation
    if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
        LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
        LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
        BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
        BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    else
    {
        printf("Error: unknown interpolation / angle representation type.\n");
        exit(1);
    }

}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
          interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;
            
          if (isGraphOne)
          {
              if (startKeyframe + frame >= 600 && startKeyframe + frame <= 800 && bone == 2)
              {
                  std::cout << interpolatedPosture.bone_rotation[bone].x() << std::endl;
              }
          }
          else if (isGraphTwo)
          {
              if (startKeyframe + frame >= 600 && startKeyframe + frame <= 800 && bone == 2)
              {
                  std::cout << interpolatedPosture.bone_rotation[bone].x() << std::endl;
              }
          }
          else if (isGraphThree)
          {
              if (startKeyframe + frame >= 200 && startKeyframe + frame <= 500 && bone == 0)
              {
                  std::cout << interpolatedPosture.bone_rotation[bone].z() << std::endl;
              }
          }
      
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this

    // Convert degrees to radians
    double x = angles[0] * M_PI / 180.0;
    double y = angles[1] * M_PI / 180.0;
    double z = angles[2] * M_PI / 180.0;

    double cx = cos(x), sx = sin(x);
    double cy = cos(y), sy = sin(y);
    double cz = cos(z), sz = sin(z);

    // R = Rz * Ry * Rx
    R[0] = cy * cz;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;

    R[3] = cy * sz;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;

    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* P1 = pInputMotion->GetPosture(startKeyframe);
        Posture* P4 = pInputMotion->GetPosture(endKeyframe);
        Posture P2;
        Posture P3;

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *P1);
        pOutputMotion->SetPosture(endKeyframe, *P4);

        // Bezier formula for each bone rotation (Euler angles)
		//calculate control points P2 and P3 for each bone, using the previous and next keyframes (if they exist)
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            Posture* prev = (startKeyframe > 0) ? pInputMotion->GetPosture(startKeyframe - 1) : P1;
            Posture* next = (endKeyframe < inputLength - 1) ? pInputMotion->GetPosture(endKeyframe + 1) : P4;

            // P1 = P0 + 1/3 * (next - prev)
            P2.bone_rotation[bone] = P1->bone_rotation[bone] + (P4->bone_rotation[bone] - prev->bone_rotation[bone]) / 3.0;
            // P2 = P3 - 1/3 * (next - prev)
            P3.bone_rotation[bone] = P4->bone_rotation[bone] - (next->bone_rotation[bone] - P1->bone_rotation[bone]) / 3.0;
        }

        // interpolate in between 
        for (int frame = 1; frame <= N; frame++)
        {
            double t = 1.0 * frame / (N + 1);
            double u = 1.0 - t;

            Posture interpolatedPosture;

            // Bezier formula for root position 
            interpolatedPosture.root_pos = P1->root_pos * (1 - t) + P4->root_pos * t;

            // Bezier formula for each bone rotation (Euler angles)
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, P1->bone_rotation[bone].p, P2.bone_rotation[bone].p,
                    P3.bone_rotation[bone].p , P4->bone_rotation[bone].p);

                if (isGraphOne)
                {
                    if (startKeyframe + frame >= 600 && startKeyframe + frame <= 800 && bone == 2)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].x() << std::endl;
                    }
                }
                else if (isGraphFour)
                {
                    if (startKeyframe + frame >= 200 && startKeyframe + frame <= 500 && bone == 0)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].z() << std::endl;
                    }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;

    Quaternion<double> prevResult[MAX_BONES_IN_ASF_FILE] = {}; // track last quaternion for each bone
    bool hasPrevResult[MAX_BONES_IN_ASF_FILE] = {};
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            Quaternion<double> q0;
            Quaternion<double> q1;
            Quaternion<double> slerpQuat;

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {   
                Euler2Quaternion( startPosture->bone_rotation[bone].p, q0);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, q1); 

				slerpQuat = Slerp(t, q0, q1);


                if (hasPrevResult[bone])
                {
                    double dot = slerpQuat.Gets() * prevResult[bone].Gets() +
                        slerpQuat.Getx() * prevResult[bone].Getx() +
                        slerpQuat.Gety() * prevResult[bone].Gety() +
                        slerpQuat.Getz() * prevResult[bone].Getz();

                    if (dot < 0.0)
                    {
                        slerpQuat = slerpQuat * -1.0;
                    }
                }
                prevResult[bone] = slerpQuat;
                hasPrevResult[bone] = true;

				Quaternion2Euler(slerpQuat, interpolatedPosture.bone_rotation[bone].p);

                for (int k = 0; k < 3; k++)
                {
                    double prevAngle = pOutputMotion->GetPosture(startKeyframe + frame - 1)->bone_rotation[bone].p[k];
                    double curr = interpolatedPosture.bone_rotation[bone].p[k];

                    // Try equivalent representations (±360)
                    while (curr - prevAngle > 180.0) curr -= 360.0;
                    while (curr - prevAngle < -180.0) curr += 360.0;

                    interpolatedPosture.bone_rotation[bone].p[k] = curr;
                }


                if (isGraphTwo)
                {
                    if (startKeyframe + frame >= 600 && startKeyframe + frame <= 800 && bone == 2)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].x() << std::endl;
                    }
                }
                else if (isGraphThree)
                {
                    if (startKeyframe + frame >= 200 && startKeyframe + frame <= 500 && bone == 0)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].z() << std::endl;
                    }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
 
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* P1 = pInputMotion->GetPosture(startKeyframe);
        Posture* P4 = pInputMotion->GetPosture(endKeyframe);
        Posture* prev = (startKeyframe > 0) ? pInputMotion->GetPosture(startKeyframe - 1) : P1;
        Posture* next = (endKeyframe < inputLength - 1) ? pInputMotion->GetPosture(endKeyframe + 1) : P4;


        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *P1);
        pOutputMotion->SetPosture(endKeyframe, *P4);
        Quaternion<double> prevResult[MAX_BONES_IN_ASF_FILE] = {}; // track last quaternion for each bone
        bool hasPrevResult[MAX_BONES_IN_ASF_FILE] = {};

        // interpolate in between 
        for (int frame = 1; frame <= N; frame++)
        {
			std::cout << "Frame: " << startKeyframe + frame << std::endl;
            double k = 0;
            if ((startKeyframe + frame) == 418)
            {
                 k = 1;
                
            }

            double t = 1.0 * frame / (N + 1);
            double u = 1.0 - t;

            Posture interpolatedPosture;

            vector prevRoot = (startKeyframe > 0) ? pInputMotion->GetPosture(startKeyframe - 1)->root_pos : P1->root_pos;
            vector nextRoot = (endKeyframe < inputLength - 1) ? pInputMotion->GetPosture(endKeyframe + 1)->root_pos : P4->root_pos;
            vector C1 = P1->root_pos + (P1->root_pos - prevRoot) / 3.0;
            vector C2 = P4->root_pos - (nextRoot - P4->root_pos) / 3.0;
            // Bezier formula for root position 
            interpolatedPosture.root_pos =
                P1->root_pos * u * u * u +
                C1 * 3 * u * u * t +
                C2 * 3 * u * t * t +
                P4->root_pos * t * t * t ;

            //interpolatedPosture.root_pos= P1->root_pos* (1 - t) + P4->root_pos * t;

            // Bezier formula for each bone rotation (Euler angles)
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
              
                // --- Convert to quaternions ---
                Quaternion<double> qPrev, qn, qn1, qNext;
                Euler2Quaternion(prev->bone_rotation[bone].p, qPrev);
                Euler2Quaternion(P1->bone_rotation[bone].p, qn);
                Euler2Quaternion(P4->bone_rotation[bone].p, qn1);
                Euler2Quaternion(next->bone_rotation[bone].p, qNext);

               /* Quaternion<double> an = Slerp(1.0 / 3.0, qn,
                    Slerp(0.5, qPrev, qn1));
                Quaternion<double> bn1 = Slerp(1.0 / 3.0, qn1,
                    Slerp(0.5, qn, qNext));*/
               Quaternion<double> anBar = Slerp(0.5, Slerp(2.0, qPrev, qn), qn1);
				Quaternion<double> an = Slerp(1.0/3.0, qn, anBar);
                Quaternion<double> bnBar = Slerp(0.5, Slerp(2.0, qn, qn1), qNext);
                Quaternion<double> bn1 = Slerp(-1.0 / 3.0, qn1, bnBar);
                //Quaternion<double> bn = Slerp(-1.0 / 3.0, qn, anBar);
    // 
    //            Quaternion<double> an =Bisect(Double(qPrev, qn),qn1);
    //            Quaternion<double> bn = Double(an, qn);
    //            Quaternion<double> an1 = Bisect(Double(qn, qn1), qNext);
				//Quaternion<double> bn1 = Double(an1, qn1);

    //            Quaternion<double> p01 = Slerp(u, qn, an);
				//Quaternion<double> p11 = Slerp(u, p01, bn1); //bn1 = p20
    //            Quaternion<double> p02 = Slerp(u, p01, p11);
				//Quaternion<double> p21 = Slerp(u, bn1, qn1);


                Quaternion<double> result = DeCasteljauQuaternion(t, qn, an, bn1, qn1);

                if (hasPrevResult[bone])
                {
                    double dot = result.Gets() * prevResult[bone].Gets() +
                        result.Getx() * prevResult[bone].Getx() +
                        result.Gety() * prevResult[bone].Gety() +
                        result.Getz() * prevResult[bone].Getz();

                    if (dot < 0.0)
                    {
                        result = result * -1.0;
                    }
                }
                prevResult[bone] = result;
                hasPrevResult[bone] = true;


                Quaternion2Euler(result, interpolatedPosture.bone_rotation[bone].p);

                if(bone ==0)
                {
                    std::cout<< interpolatedPosture.bone_rotation[bone].p[0]<<
						" " << interpolatedPosture.bone_rotation[bone].p[1] <<
						" " << interpolatedPosture.bone_rotation[bone].p[2] << std::endl;
                }

                if (isGraphTwo)
                {
                    if (startKeyframe + frame >= 600 && startKeyframe + frame <= 800 && bone == 2)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].x() << std::endl;
                    }
                }
                else if (isGraphFour)
                {
                    if (startKeyframe + frame >= 200 && startKeyframe + frame <= 500 && bone == 0)
                    {
                        std::cout << interpolatedPosture.bone_rotation[bone].z() << std::endl;
                    }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
  
    // Convert degrees to radians
    double x = angles[0] * M_PI / 180.0;
    double y = angles[1] * M_PI / 180.0;
    double z = angles[2] * M_PI / 180.0;

    // Compute half angles
    double cx = cos(x * 0.5);
    double sx = sin(x * 0.5);
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cz = cos(z * 0.5);
    double sz = sin(z * 0.5);

    // Quaternion (s, x, y, z)
    //double qs = cz * cy * cx - sz * sy * sx; // s
    //double qx = cz * cy * sx + sz * sy * cx; // x
    //double qy = cz * sy * cx - sz * cy * sx; // y
    //double qz = sz * cy * cx + cz * sy * sx; // z

    double qs = cx * cy * cz + sx * sy * sz;
    double qx = sx * cy * cz - cx * sy * sz;
    double qy = cx * sy * cz + sx * cy * sz;
    double qz = cx * cy * sz - sx * sy * cz;

	q.Set(qs, qx, qy, qz); // identity quaternion
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
	double R[9];
    double s = q.Gets();
    double x = q.Getx();
    double y = q.Gety();
    double z = q.Getz();
    // Normalize quaternion (important!)
    //double norm = sqrt(s * s + x * x + y * y + z * z);
    //s /= norm;
    //x /= norm;
    //y /= norm;
    //z /= norm;


    // Quaternion → Rotation Matrix (row-major)
    R[0] = 1 - 2 * y * y - 2 * z * z;
    R[1] = 2 * x * y - 2 * z * s;
    R[2] = 2 * x * z + 2 * y * s;

    R[3] = 2 * x * y + 2 * z * s;
    R[4] = 1 - 2 * x * x - 2 * z * z;
    R[5] = 2 * y * z - 2 * x * s;

    R[6] = 2 * x * z - 2 * y * s;
    R[7] = 2 * y * z + 2 * x * s;
    R[8] = 1 - 2 * x * x - 2 * y * y;


	Rotation2Euler(R, angles);


}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this

    Quaternion<double> result;
  Quaternion<double> qEnd = qEnd_;

    double dot = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() +
         qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();

    if (dot < 0) {

        qEnd = qEnd * -1.0;
        dot = -dot;

    }

    double  s = 1 - t;

    if (dot < 0.9995) {

        // slerp

        double theta = acos(dot);
        double angleSin = sin(theta);

        s = sin(s * theta) / angleSin;
        t = sin(t * theta) / angleSin;

            result = qStart * s + qEnd  * t;


    }
    else {

        // for small angles, lerp then normalize

        result = qStart * s + qEnd * t;

        result.Normalize(); // normalize calls _onChangeCallback()

    }

    return result;
  //Quaternion<double> result;
  //Quaternion<double> qEnd = qEnd_;
  //// Calculate angle between them.
  //double cosHalfTheta = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() +
  //    qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();

  //// if qa=qb or qa=-qb then theta = 0 and we can return qa 
  //if (cosHalfTheta < 0.0)
  //{
  //    qEnd = qEnd * -1.0;
  //    cosHalfTheta = -cosHalfTheta;
  //}
  //else if(cosHalfTheta >= 1.0)
  //{
  //    return qStart;
  //}

  //// Calculate temporary values.
  //double halfTheta = acos(cosHalfTheta);
  //double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);

  //// if theta = 180 degrees then result is not fully defined
  //// we could rotate around any axis normal to qa or qb
  //if (fabs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
	 // result = qStart * 0.5 + qEnd * 0.5;
  //    return result;
  //}

  //double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
  //double ratioB = sin(t * halfTheta) / sinHalfTheta;
  ////calculate Quaternion.
  //result = ratioA * qStart + ratioB * qEnd;
  //result.Normalize();
  //return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
    Quaternion<double> result;
    Quaternion<double> dot;
    Quaternion<double> subt;

    dot = Dot(p, q);
    subt = p - q;
    result = 2 * dot * subt;

    return result;
}

Quaternion<double> Interpolator::Bisect(Quaternion<double> p, Quaternion<double> q)
{
    // students should implement this
    Quaternion<double> result;
    result = p+ q;
    result.Normalize();

    return result;
}

double Interpolator::Dot(Quaternion<double> p, Quaternion<double> q)
{
    // students should implement this
    double result= p.Gets()* q.Gets() + p.Getx() * q.Getx() +
        p.Gety() * q.Gety() + p.Getz() * q.Getz();

    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  Quaternion<double> q0, q1, q2, r0, r1, pt;
  Quaternion<double> point0, point1, point2, point3;
  
  Euler2Quaternion(p0.p, point0);
  Euler2Quaternion(p1.p, point1);
  Euler2Quaternion(p2.p, point2);
  Euler2Quaternion(p3.p, point3);

  q0 = Slerp(t, point0, point1);
  q1 = Slerp(t, point1, point2);
  q2 = Slerp(t, point2, point3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);
  pt = Slerp(t, r0, r1);

  pt.Normalize();
  Quaternion2Euler(pt, result.p);
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> q0, q1, q2, r0, r1, pt;

  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);
  pt = Slerp(t, r0, r1);

  pt.Normalize();
  result = pt;
  return result;
}

