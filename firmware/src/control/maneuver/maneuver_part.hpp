#ifndef MANEUVER_PART_HPP_
#define MANEUVER_PART_HPP_

namespace control
{

/* maneuver floating point type */
typedef double mnrfp;

enum class ManeuverPartType
{
  line,
  arc,
  unknown
};

struct ManeuverLine
{
  mnrfp start[2][1];  /* local north (m), east (m) coordinates */
  mnrfp finish[2][1]; /* local north (m), east (m) coordinates */
};

struct ManeuverArc
{
  mnrfp center[2][1]; /* local north (m), east (m) coordinates */
  mnrfp radius;       /* arc radius (m), if positive - clockwise, else - counter-clockwise */
  mnrfp startCourse;  /* course (rad) on arc start, [0, 2*M_PI] */
  mnrfp deltaCourse;  /* course change (rad) on arc, [0; M_PI] */
};

struct execOut
{
  execOut(void) :
    dz(0),
    dist(0),
    crs(0) ,
    alt(0),
    final(true),
    crossed(true)
  {;}

  execOut(
      mnrfp dz,
      mnrfp dist,
      mnrfp crs,
      mnrfp alt,
      bool final,
      bool crossed) :
    dz(dz),
    dist(dist),
    crs(crs),
    alt(alt),
    final(final),
    crossed(crossed)
  {;}

  mnrfp dz;     /* cross track error (m) */
  mnrfp dist;   /* distance to target point (m) */
  mnrfp crs;    /* course to target point (rad), [0; 2*M_PI] */
  mnrfp alt;    /* target altitude (m, WGS-84) */
  bool final;   /* is mission part last in maneuver */
  bool crossed; /* is mission part crossed  */
};

class ManeuverPart
{
public:
  ManeuverPart();
  void fillLine(
      const mnrfp (&start)[2][1],
      const mnrfp (&finish)[2][1]);
  void fillLine(
      mnrfp startNorth,
      mnrfp startEast,
      mnrfp finishNorth,
      mnrfp finishEast);
  void fillArc(
      const mnrfp (&center)[2][1],
      mnrfp radius,
      mnrfp startCourse,
      mnrfp deltaCourse);
  void fillArc(
      mnrfp centerNorth,
      mnrfp centerEast,
      mnrfp radius,
      mnrfp startCourse,
      mnrfp deltaCourse);
  void fillAlt(mnrfp alt);
  void fillUnknown();
  void setFinal(bool final);
  ManeuverPartType type() const;
  ManeuverArc getArc() const;
  ManeuverLine getLine() const;
  bool isFinal() const;
  void rotate(mnrfp angle);
  void flipNorth();
  void flipEast();
  void move(const mnrfp (&localDelta)[2][1]);
  void execute(execOut &out) const;

private:
  void executeLine(execOut &out) const;
  void executeArc(execOut &out) const;
  void executeUnknown(execOut &out) const;

  ManeuverPartType partType; /* type of maneuver part */
  bool final;                /* is last part in maneuver */
  ManeuverArc arc;
  ManeuverLine line;
  mnrfp alt;

};

} /* namespace control */

#endif /* MANEUVER_PART_HPP_ */
