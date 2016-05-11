#include <math.h>
#include <string.h>
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control
{

/*
 * TODO: add comment
 */
void ManeuverPart::executeLine(execOut &out) const
{
  out.alt = alt;
  out.final = final;

  mnrfp lineVector[2][1];
  m_minus<mnrfp, 2, 1>(lineVector, line.finish, line.start);
  mnrfp lineVectorNorm[2][1];
  m_copy<mnrfp, 2, 1>(lineVectorNorm, lineVector);
  m_norm<mnrfp, 2>(lineVectorNorm);
  out.crs = atan2(lineVectorNorm[1][0], lineVectorNorm[0][0]);

  mnrfp currToFinishVectorTrans[1][2];
  m_tran<mnrfp, 2, 1>(currToFinishVectorTrans, line.finish);
  mnrfp signMtr[1][1];
  m_mul<mnrfp, 1, 2, 1>(
      signMtr,
      currToFinishVectorTrans,
      lineVectorNorm);
  mnrfp sgn = sign<mnrfp>(signMtr[0][0]);
  out.dist = sgn * m_vec_norm<mnrfp, 2>(line.finish);
  if (out.dist < static_cast<mnrfp>(0.0))
  {
    out.crossed = true;
  }
  else
  {
    out.crossed = false;
  }

  mnrfp perpendVector[2][1] = {
      {-lineVectorNorm[1][0]},
      { lineVectorNorm[0][0]}};
  m_norm<mnrfp, 2>(perpendVector);
  mnrfp perpendVectorTrans[1][2];
  m_tran<mnrfp, 2, 1>(perpendVectorTrans, perpendVector);
  mnrfp currToFinishVectorMinus[2][1];
  m_mul_s<mnrfp, 2, 1>(
      currToFinishVectorMinus,
      line.finish,
      -1.0);
  mnrfp dZ[1][1];
  m_mul<mnrfp, 1, 2, 1>(
      dZ,
      perpendVectorTrans,
      currToFinishVectorMinus);
  out.dz = dZ[0][0];
}

/*
 * TODO: add comment
 */
void ManeuverPart::executeArc(execOut &out) const
{
  out.alt = alt;
  out.final = final;

  mnrfp cwSign = sign<mnrfp>(arc.radius);
  mnrfp centerToCurrVector[2][1];

  m_mul_s<mnrfp, 2, 1>(centerToCurrVector, arc.center, -1.0);
  out.dist = m_vec_norm<mnrfp, 2>(centerToCurrVector);
  out.dz = cwSign * (fabs(arc.radius) - out.dist);
  out.crs = tangentLineCourse(centerToCurrVector, cwSign);
  mnrfp deltaCourse = cwSign * (out.crs - arc.startCourse);
  deltaCourse = wrap_pi(deltaCourse);
  if (fabs(deltaCourse) >= arc.deltaCourse)
  {
    out.crossed = true;
  }
  else
  {
    out.crossed = false;
  }
}

void ManeuverPart::executeUnknown(execOut &out) const
{
  out.dz = 0.0;
  out.dist = 0.0;
  out.crs = 0.0;
  out.final = true;
  out.crossed = true;
  out.alt = 0.0;
}

ManeuverPart::ManeuverPart()
{
  memset(this, 0, sizeof(ManeuverPart));
  final = true;
  partType = ManeuverPartType::unknown;
}

/*
 * Clear arc data, change active part type to line and fill its data.
 */
void ManeuverPart::fillLine(
    const mnrfp (&start)[2][1],
    const mnrfp (&finish)[2][1])
{
  m_copy<mnrfp, 2, 1>(line.start, start);
  m_copy<mnrfp, 2, 1>(line.finish, finish);
  memset(&(this->arc), 0, sizeof(ManeuverArc));
  partType = ManeuverPartType::line;
}

void ManeuverPart::fillLine(
    mnrfp startNorth,
    mnrfp startEast,
    mnrfp finishNorth,
    mnrfp finishEast)
{
  line.start[0][0] = startNorth;
  line.start[1][0] = startEast;
  line.finish[0][0] = finishNorth;
  line.finish[1][0] = finishEast;
  memset(&(this->arc), 0, sizeof(ManeuverArc));
  partType = ManeuverPartType::line;
}

void ManeuverPart::fillArc(
    const mnrfp (&center)[2][1],
    mnrfp radius,
    mnrfp startCourse,
    mnrfp deltaCourse)
{
  m_copy<mnrfp, 2, 1>(arc.center, center);
  arc.radius = radius;
  arc.startCourse = startCourse;
  arc.deltaCourse = deltaCourse;
  memset(&(this->line), 0, sizeof(ManeuverLine));
  partType = ManeuverPartType::arc;
}

void ManeuverPart::fillArc(
    mnrfp centerNorth,
    mnrfp centerEast,
    mnrfp radius,
    mnrfp startCourse,
    mnrfp deltaCourse)
{
  arc.center[0][0] = centerNorth;
  arc.center[1][0] = centerEast;
  arc.radius = radius;
  arc.startCourse = startCourse;
  arc.deltaCourse = deltaCourse;
  memset(&(this->line), 0, sizeof(ManeuverLine));
  partType = ManeuverPartType::arc;
}

void ManeuverPart::fillAlt(mnrfp alt)
{
  this->alt = alt;
}

void ManeuverPart::fillUnknown()
{
  memset(&(this->line), 0, sizeof(ManeuverLine));
  memset(&(this->arc), 0, sizeof(ManeuverArc));
  partType = ManeuverPartType::unknown;
  final = true;
}

void ManeuverPart::setFinal(bool final)
{
  this->final = final;
}

ManeuverPartType ManeuverPart::type() const
{
  return partType;
}

ManeuverArc ManeuverPart::getArc() const
{
  return arc;
}

ManeuverLine ManeuverPart::getLine() const
{
  return line;
}

bool ManeuverPart::isFinal() const
{
  return final;
}

void ManeuverPart::rotate(mnrfp angle)
{
  mnrfp sinAngle = sin(angle);
  mnrfp cosAngle = cos(angle);
  mnrfp dcm[2][2] = {
      {cosAngle, -sinAngle},
      {sinAngle,  cosAngle}};
  mnrfp tmp[2][1];

  switch (partType)
  {
    case ManeuverPartType::line:
      m_mul<mnrfp, 2, 2, 1>(tmp, dcm, line.start);
      m_copy<mnrfp, 2, 1>(line.start, tmp);
      m_mul<mnrfp, 2, 2, 1>(tmp, dcm, line.finish);
      m_copy<mnrfp, 2, 1>(line.finish, tmp);
      break;

    case ManeuverPartType::arc:
      m_mul<mnrfp, 2, 2, 1>(tmp, dcm, arc.center);
      m_copy<mnrfp, 2, 1>(arc.center, tmp);
      arc.startCourse = wrap_2pi(arc.startCourse + angle);
      break;

    default:
      break;
  }
}

void ManeuverPart::flipNorth()
{
  switch (partType)
  {
    case ManeuverPartType::line:
      line.start[1][0] = -line.start[1][0];
      line.finish[1][0] = -line.finish[1][0];
      break;

    case ManeuverPartType::arc:
      arc.center[1][0] = -arc.center[1][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(
          static_cast<mnrfp>(2.0 * M_PI)
        - arc.startCourse);
      break;

    default:
      break;
  }
}

void ManeuverPart::flipEast()
{
  switch (partType)
  {
    case ManeuverPartType::line:
      line.start[0][0] = -line.start[0][0];
      line.finish[0][0] = -line.finish[0][0];
      break;

    case ManeuverPartType::arc:
      arc.center[0][0] = -arc.center[0][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(
          static_cast<mnrfp>(M_PI)
        - arc.startCourse);
      break;

    default:
      break;
  }
}

void ManeuverPart::move(const mnrfp (&localDelta)[2][1])
{
  switch (partType)
  {
    case ManeuverPartType::arc:
      m_plus<mnrfp, 2, 1>(arc.center, arc.center, localDelta);
      break;

    case ManeuverPartType::line:
      m_plus<mnrfp, 2, 1>(line.start, line.start, localDelta);
      m_plus<mnrfp, 2, 1>(line.finish, line.finish, localDelta);
      break;

    default:
      break;
  }
}

void ManeuverPart::execute(execOut &out) const
{
  switch(partType)
  {
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
