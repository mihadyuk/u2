#ifndef MANEUVER_PART_HPP_
#define MANEUVER_PART_HPP_

#include <math.h>
#include <string.h>
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control {

enum class ManeuverPartType {
  line,
  arc,
  unknown
};

template <typename T>
struct ManeuverLine {
  T start[2][1];  /* local north (m), east (m) coordinates */
  T finish[2][1]; /* local north (m), east (m) coordinates */
};

template <typename T>
struct ManeuverArc {
  T center[2][1]; /* local north (m), east (m) coordinates */
  T radius;       /* arc radius (m), if positive - clockwise, else - counter-clockwise */
  T startCourse;  /* course (rad) on arc start, [0, 2*M_PI] */
  T deltaCourse;  /* course change (rad) on arc, [0; M_PI] */
};

template <typename T>
struct partExecOut {
  partExecOut(void) :
    dz(0),
    dist(0),
    crs(0) ,
    alt(0),
    final(true),
    crossed(true) {;}

  partExecOut(T dz,
              T dist,
              T crs,
              T alt,
              bool final,
              bool crossed) :
    dz(dz),
    dist(dist),
    crs(crs),
    alt(alt),
    final(final),
    crossed(crossed) {;}

  T dz;         /* cross track error (m) */
  T dist;       /* distance to target point (m) */
  T crs;        /* course to target point (rad), [0; 2*M_PI] */
  T alt;        /* target altitude (m, WGS-84) */
  bool final;   /* is mission part last in maneuver */
  bool crossed; /* is mission part crossed  */
};

template <typename T>
class ManeuverPart {

public:
  ManeuverPart();

  void fillLine(const T (&start)[2][1],
                const T (&finish)[2][1]);

  void fillLine(T startNorth,
                T startEast,
                T finishNorth,
                T finishEast);

  void fillArc(const T (&center)[2][1],
               T radius,
               T startCourse,
               T deltaCourse);

  void fillArc(T centerNorth,
               T centerEast,
               T radius,
               T startCourse,
               T deltaCourse);

  void fillAlt(T alt);
  void fillUnknown();
  void setFinal(bool final);
  ManeuverPartType type() const;
  ManeuverArc<T> getArc() const;
  ManeuverLine<T> getLine() const;
  bool isFinal() const;
  void rotate(T angle);
  void flipNorth();
  void flipEast();
  void move(const T (&localDelta)[2][1]);
  void execute(partExecOut<T> &out) const;

private:
  void executeLine(partExecOut<T> &out) const;
  void executeArc(partExecOut<T> &out) const;
  void executeUnknown(partExecOut<T> &out) const;

  ManeuverPartType partType; /* type of maneuver part */
  bool final;                /* is last part in maneuver */
  ManeuverArc<T> arc;
  ManeuverLine<T> line;
  T alt;

};

/*
 * TODO: add comment
 */
template <typename T>
void ManeuverPart<T>::executeLine(partExecOut<T> &out) const {
  out.alt = alt;
  out.final = final;


  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, line.finish, line.start);

  T lineVectorNorm[2][1];
  m_copy<T, 2, 1>(lineVectorNorm, lineVector);
  m_norm<T, 2>(lineVectorNorm);
  out.crs = atan2(lineVectorNorm[1][0], lineVectorNorm[0][0]);

  T currToFinishVectorTrans[1][2];
  m_tran<T, 2, 1>(currToFinishVectorTrans, line.finish);

  T signMtr[1][1];
  m_mul<T, 1, 2, 1>(signMtr, currToFinishVectorTrans, lineVectorNorm);
  T sgn = sign<T>(signMtr[0][0]);

  out.dist = sgn*m_vec_norm<T, 2>(line.finish);
  if (out.dist < static_cast<T>(0.0)) {
    out.crossed = true;
  } else {
    out.crossed = false;
  }

  T perpendVector[2][1] = {{-lineVectorNorm[1][0]},
                           { lineVectorNorm[0][0]}};
  m_norm<T, 2>(perpendVector);
  T perpendVectorTrans[1][2];
  m_tran<T, 2, 1>(perpendVectorTrans, perpendVector);

  T currToFinishVectorMinus[2][1];
  m_mul_s<T, 2, 1>(currToFinishVectorMinus, line.finish, -1.0);
  T dZ[1][1];
  m_mul<T, 1, 2, 1>(dZ, perpendVectorTrans, currToFinishVectorMinus);
  out.dz = dZ[0][0];
}

/*
 * TODO: add comment
 */
template <typename T>
void ManeuverPart<T>::executeArc(partExecOut<T> &out) const {
  out.alt = alt;
  out.final = final;

  T cwSign = sign<T>(arc.radius);
  T centerToCurrVector[2][1];
  m_mul_s<T, 2, 1>(centerToCurrVector, arc.center, -1.0);

  out.dist = m_vec_norm<T, 2>(centerToCurrVector);
  out.dz = cwSign*(fabs(arc.radius) - out.dist);
  out.crs = tangentLineCourse(centerToCurrVector, cwSign);

  T deltaCourse = cwSign*(out.crs - arc.startCourse);
  deltaCourse = wrap_pi(deltaCourse);
  if (fabs(deltaCourse) >= arc.deltaCourse) {
    out.crossed = true;
  } else {
    out.crossed = false;
  }
}

template <typename T>
void ManeuverPart<T>::executeUnknown(partExecOut<T> &out) const {
  out.dz = 0.0;
  out.dist = 0.0;
  out.crs = 0.0;
  out.final = true;
  out.crossed = true;
  out.alt = -1.0;
}

template <typename T>
ManeuverPart<T>::ManeuverPart() {

  memset(this, 0, sizeof(ManeuverPart<T>));
  final = true;
  partType = ManeuverPartType::unknown;
}

/*
 * Clear arc data, change active part type to line and fill its data.
 */
template <typename T>
void ManeuverPart<T>::fillLine(const T (&start)[2][1],
                               const T (&finish)[2][1]) {
  m_copy<T, 2, 1>(line.start, start);
  m_copy<T, 2, 1>(line.finish, finish);
  memset(&(this->arc), 0, sizeof(ManeuverArc<T>));
  partType = ManeuverPartType::line;
}

template <typename T>
void ManeuverPart<T>::fillLine(T startNorth,
                               T startEast,
                               T finishNorth,
                               T finishEast) {
  line.start[0][0] = startNorth;
  line.start[1][0] = startEast;
  line.finish[0][0] = finishNorth;
  line.finish[1][0] = finishEast;
  memset(&(this->arc), 0, sizeof(ManeuverArc<T>));
  partType = ManeuverPartType::line;
}

template <typename T>
void ManeuverPart<T>::fillArc(const T (&center)[2][1],
                              T radius,
                              T startCourse,
                              T deltaCourse) {
  m_copy<T, 2, 1>(arc.center, center);
  arc.radius = radius;
  arc.startCourse = startCourse;
  arc.deltaCourse = deltaCourse;
  memset(&(this->line), 0, sizeof(ManeuverLine<T>));
  partType = ManeuverPartType::arc;
}

template <typename T>
void ManeuverPart<T>::fillArc(T centerNorth,
                              T centerEast,
                              T radius,
                              T startCourse,
                              T deltaCourse) {
  arc.center[0][0] = centerNorth;
  arc.center[1][0] = centerEast;
  arc.radius = radius;
  arc.startCourse = startCourse;
  arc.deltaCourse = deltaCourse;
  memset(&(this->line), 0, sizeof(ManeuverLine<T>));
  partType = ManeuverPartType::arc;
}

template <typename T>
void ManeuverPart<T>::fillAlt(T alt) {
  this->alt = alt;
}

template <typename T>
void ManeuverPart<T>::fillUnknown() {
  memset(&(this->line), 0, sizeof(ManeuverLine<T>));
  memset(&(this->arc), 0, sizeof(ManeuverArc<T>));
  partType = ManeuverPartType::unknown;
  final = true;
}

template <typename T>
void ManeuverPart<T>::setFinal(bool final) {
  this->final = final;
}

template <typename T>
ManeuverPartType ManeuverPart<T>::type() const {
  return partType;
}

template <typename T>
ManeuverArc<T> ManeuverPart<T>::getArc() const {
  return arc;
}

template <typename T>
ManeuverLine<T> ManeuverPart<T>::getLine() const {
  return line;
}

template <typename T>
bool ManeuverPart<T>::isFinal() const {
  return final;
}

template <typename T>
void ManeuverPart<T>::rotate(T angle) {
  T sinAngle = sin(angle);
  T cosAngle = cos(angle);
  T dcm[2][2] = {{cosAngle, -sinAngle},
                 {sinAngle,  cosAngle}};
  T tmp[2][1];

  switch (partType) {
    case ManeuverPartType::line: {
      m_mul<T, 2, 2, 1>(tmp, dcm, line.start);
      m_copy<T, 2, 1>(line.start, tmp);
      m_mul<T, 2, 2, 1>(tmp, dcm, line.finish);
      m_copy<T, 2, 1>(line.finish, tmp);
      break;
    }
    case ManeuverPartType::arc: {
      m_mul<T, 2, 2, 1>(tmp, dcm, arc.center);
      m_copy<T, 2, 1>(arc.center, tmp);
      arc.startCourse = wrap_2pi(arc.startCourse + angle);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::flipNorth() {
  switch (partType) {
    case ManeuverPartType::line: {
      line.start[1][0] = -line.start[1][0];
      line.finish[1][0] = -line.finish[1][0];
      break;
    }
    case ManeuverPartType::arc: {
      arc.center[1][0] = -arc.center[1][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(static_cast<T>(2.0*M_PI) - arc.startCourse);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::flipEast() {
  switch (partType) {
    case ManeuverPartType::line: {
      line.start[0][0] = -line.start[0][0];
      line.finish[0][0] = -line.finish[0][0];
      break;
    }
    case ManeuverPartType::arc: {
      arc.center[0][0] = -arc.center[0][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(static_cast<T>(M_PI) - arc.startCourse);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::move(const T (&localDelta)[2][1]) {
  switch (partType) {
    case ManeuverPartType::arc: {
      m_plus<T, 2, 1>(arc.center, arc.center, localDelta);
      break;
    }
    case ManeuverPartType::line: {
      m_plus<T, 2, 1>(line.start, line.start, localDelta);
      m_plus<T, 2, 1>(line.finish, line.finish, localDelta);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::execute(partExecOut<T> &out) const {
  switch(partType) {
    case ManeuverPartType::line:
      executeLine(out);
      break;

    case ManeuverPartType::arc:
      executeArc(out);
      break;

    case ManeuverPartType::unknown:
      executeUnknown(out);
      break;
  }
}

} /* namespace control */

#endif /* MANEUVER_PART_HPP_ */
