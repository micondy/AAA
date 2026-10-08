#include "mobile_base/bounded_motion.hpp"
#include <iostream>
#include <stdexcept>
using M = mobile_base::BoundedMotion;
void check(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
bool near(double a, double b) {return std::abs(a-b) < 1e-8;}
int main()
{
  // 每个合法目标必须还原原速度矢量，不允许为美观或越限单独反号。
  for (int n = -180; n <= 180; ++n) {
    double a = n*M::pi/180, q=0, v=0;
    check(M::target(std::cos(a), std::sin(a), 0.4, 0.4, M::pi/2, q, v), "missing feasible angle");
    check(std::abs(q) <= M::pi/2, "target outside limit");
    check(near(v*std::cos(q),std::cos(a)) && near(v*std::sin(q),std::sin(a)), "vector mismatch");
  }
  double q=0,v=0;
  check(!M::target(0,1,0,0,0.5,q,v), "unreachable angle should not clamp");
  check(M::target(0,1,0,0,M::pi/2,q,v,1,5*M::pi/180), "left lateral missing");
  check(near(q,M::pi/2) && near(v,1), "left lateral preference failed");
  check(M::target(0,1,0,0,M::pi/2,q,v,-1,5*M::pi/180), "right lateral missing");
  check(near(q,-M::pi/2) && near(v,-1), "right lateral preference failed");
  check(M::target(0,1,10*M::pi/180,0,M::pi/2,q,v,-1,5*M::pi/180),
    "nearest lateral missing");
  check(near(q,M::pi/2) && near(v,1), "preference overrode clearly shorter path");
  for(double current : {-M::pi/2,M::pi/2}) {
    check(M::target(0,-1,current,current,M::pi/2,q,v), "reverse lateral missing");
    check(near(q,current) && near(v*std::sin(q),-1), "lateral reversal turns wheel");
  }
  M m; M::Four actual{}, stopped{}, rolling{2,2,2,2};
  check(m.reset(actual),"reset failed");
  check(m.step(0,1,0,actual,rolling,0.01),"brake failed");
  check(m.braking && near(m.angle[0],0) && near(m.speed[0],0),"steered before stopped");
  for(int i=0;i<10;++i) m.step(0,1,0,actual,stopped,0.01);
  check(near(m.angle[0],0),"did not wait stable stop");
  for(int i=0;i<10;++i) m.step(0,1,0,actual,stopped,0.01);
  check(m.angle[0]<0 && -m.angle[0] <= 0.03,"steering slew failed");
  m.step(0,0,0,actual,stopped,0.01);
  check(near(m.angle[0],actual[0]) && near(m.speed[0],0),"stop did not cancel steering");
  // 边界 +89° 到等价 -89°：先停车，并沿合法区间逐步转，不瞬间跳变。
  actual.fill(89*M::pi/180); m.reset(actual);
  double a=91*M::pi/180;
  m.step(std::cos(a),std::sin(a),0,actual,rolling,0.01);
  check(near(m.angle[0],actual[0]) && m.braking,"boundary moved before stop");
  for(int i=0;i<20;++i) m.step(std::cos(a),std::sin(a),0,actual,stopped,0.01);
  check(m.angle[0]<actual[0] && actual[0]-m.angle[0]<0.03,"boundary target jumped");
  actual.fill(0); m.reset(actual);
  m.step(1,0,0,actual,stopped,0.01);
  check(near(m.angle[0],0) && near(m.speed[0],0.08),"acceleration limit failed");
  m.step(2,0,0,actual,stopped,0.01);
  check(near(m.angle[0],0) && near(m.speed[0],0.16),"speed scaling changed angle");
  actual[0]=2*M::pi;
  check(!m.step(1,0,0,actual,stopped,0.01) && near(m.speed[0],0),"wrapped multi-turn feedback");
  actual[0]=std::numeric_limits<double>::quiet_NaN();
  check(!m.reset(actual),"accepted NaN");
  // 用理想关节跟随跑完整过程，验证对准前不放行、对准后能恢复。
  actual.fill(0); stopped.fill(0); check(m.reset(actual), "second reset failed");
  for(int i=0;i<1400;++i) {
    const auto previous=m.angle;
    check(m.step(0,0.2,0,actual,stopped,0.01), "full transition failed");
    for(size_t j=0;j<4;++j) {
      check(std::abs(m.angle[j]-previous[j])<=0.0015+1e-10, "slew exceeded");
      const double expected=(j==1||j==2)?M::pi/2:-M::pi/2;
      if(std::abs(expected-actual[j])>m.alignment) check(near(m.speed[j],0), "drive before alignment");
    }
    actual=m.angle; stopped=m.speed;
  }
  check(near(m.angle[0],-M::pi/2) && near(m.angle[1],M::pi/2) &&
    near(m.angle[2],M::pi/2) && near(m.angle[3],-M::pi/2) &&
    near(m.speed[0],-2) && near(m.speed[1],2) &&
    near(m.speed[2],2) && near(m.speed[3],-2),
    "never resumed outward-caster lateral drive");
  for(int i=0;i<100;++i) {
    check(m.step(0,-0.2,0,actual,stopped,0.01), "reverse failed");
    check(near(m.angle[0],-M::pi/2), "reverse changed angle");
    actual=m.angle; stopped=m.speed;
  }
  check(near(m.speed[0],2), "reverse did not finish");
  std::cout << "Bounded motion regression checks passed\n";
}
