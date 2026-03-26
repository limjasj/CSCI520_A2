#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

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

    double eulerAngles[3] = {-45, 30, 60 };
    double rotationMatrix[9] = { 0.5, -.5,.707, -.5, .5, .707 , -.707, -.707, 0 };
    Quaternion<double> quat(0.723317, 0.439680, 0.022260, 0.531976);
  
    //Euler2Rotation(eulerAngles, rotationMatrix);
    //Rotation2Euler(rotationMatrix, eulerAngles);

    double R1[9], R2[9];
    double out[3];
    double input[3] = { 90,45,-30 };

    Euler2Quaternion(eulerAngles, quat);
    Quaternion2Euler(quat, eulerAngles);

  //Euler2Quaternion(eulerAngles, quat);
  //Euler2Rotation(eulerAngles, rotationMatrix);

  Rotation2Euler(rotationMatrix, eulerAngles);
  Euler2Quaternion(eulerAngles, quat);
  Quaternion2Euler(quat, eulerAngles);

  Quaternion2Euler(quat, eulerAngles);
  Euler2Rotation(eulerAngles, rotationMatrix);
  Rotation2Euler(rotationMatrix, eulerAngles);
  Euler2Quaternion(eulerAngles, quat);

  //Euler2Quaternion(eulerAngles, quat);
  //Quaternion2Euler(quat, eulerAngles);

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
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

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
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
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
    double qs = cz * cy * cx - sz * sy * sx; // s
    double qx = cz * cy * sx + sz * sy * cx; // x
    double qy = cz * sy * cx - sz * cy * sx; // y
    double qz = sz * cy * cx + cz * sy * sx; // z

	q.Set(qs, qx, qy, qz); // identity quaternion
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
  
	double R[9];
    double s = q.Gets();
    double x = q.Getx();
    double y = q.Gety();
    double z = q.Getz();
    // Normalize quaternion (important!)
    double norm = sqrt(s * s + x * x + y * y + z * z);
    s /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    // Quaternion → Rotation Matrix (row-major)
    R[0] = 1 - 2 * y * y - 2 * z * z;
    R[1] = 2 * x * y + 2 * z * s;
    R[2] = 2 * x * z - 2 * y * s;

    R[3] = 2 * x * y - 2 * z * s;
    R[4] = 1 - 2 * x * x - 2 * z * z;
    R[5] = 2 * y * z + 2 * x * s;

    R[6] = 2 * x * z + 2 * y * s;
    R[7] = 2 * y * z - 2 * x * s;
    R[8] = 1 - 2 * x * x - 2 * y * y;

	Rotation2Euler(R, angles);

    angles[0] *= -1.0;
    angles[1] *= -1.0;
    angles[2] *= -1.0;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  double dot = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() +
      qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

  // Shortest path
  if (dot < 0.0)
  {
      dot = -dot;
      qEnd_= qEnd_ * -1;
  }

  // Clamp
  if (dot > 1.0) dot = 1.0;
  if (dot < -1.0) dot = -1.0;

  // Angle
  double theta = acos(dot);

  double q1Coeff = sin((1 - t)* theta)/ sin(theta);
  double q2Coeff = sin(t * theta) / sin(theta);
  result = (q1Coeff * qStart) + (q2Coeff * qEnd_);

  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> mult;
  Quaternion<double> subt;

  mult = p * q;
  mult = 2 * mult;
  subt = p - q;
  result = mult * subt;

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

  return result;
}

