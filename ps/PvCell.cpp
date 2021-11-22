#include "PvCell.hpp"
#include <math.h>
#include <spdlog/spdlog.h>

double PvCell::dio_iret(double v, double is1, double n) {
  const double k_Boltz = 1.38e-23;
  const double q_Electron = 1.6e-19;
  const double T1 = 273 + 25;
  return is1 * (exp(q_Electron * v / (n * k_Boltz * T1)) - 1);
}

double PvCell::pv(const double lux, const double iload) {
  double rp_scale;
  if (lux > 1800) {
    rp_scale = 2.0;
  } else {
    rp_scale = 1.0;
  }

  double iph = -9e-12 * lux * lux + 70e-9 * lux - 5e-6;
  double is1 = 5e9 * pow(lux, -9.5);
  // double is2= 9e27 * pow(lux, -20.54);
  double rp = rp_scale * 3e6 * pow(lux, -0.57);
  double rs = 3e6 * pow(lux, -0.78);

  // Use one-diode model (no significant change in accuracy)

  // Start at a good approximation
  double vterm = 0.6;
  double n1 = 1.915;
  double vnew = 0.0;

  // Work your way up the curve until the guess converges
  while (vterm > 0.0) {
    double id1 = PvCell::dio_iret(vterm, is1, n1);
    vnew = (iph - iload - id1) * rp;
    if (vnew > (vterm + 1.0e-5)) {
      vterm += 0.0001;
    } else {
      vterm = 0.0;
    }
  }
  double vout = vnew - iload * rs;
  // Return zero if drop across rs results in negative values
  return vout < 0.0 ? 0.0 : vout;
}

double PvCell::singleDiodeEquation(const double lux, const double vload) {
  double rp_scale;
  if (lux > 1800) {
    rp_scale = 2.0;
  } else {
    rp_scale = 1.0;
  }

  double iph = -9e-12 * lux * lux + 70e-9 * lux - 5e-6; // Photo current
  double is1 = 5e9 * pow(lux, -9.5);            // Diode saturation current?
  double rp = rp_scale * 3e6 * pow(lux, -0.57); // Parallel resistance
  double rs = 3e6 * pow(lux, -0.78);            // Series resistance
  double n1 = 1.915;                            // Diode factor

  // Source of the following single diode equation:
  // https://www.researchgate.net/publication/281942470_Single-Diode_and_Two-Diode_Pv_Cell_Modeling_Using_Matlab_For_Studying_Characteristics_Of_Solar_Cell_Under_Varying_Conditions

  // Keep guessing what the current is until we get
  // iguess == iph - id - ip
  // where
  //   iph      is the photocurrent
  //   id       is the diode current
  //   ip       is the current through rp
  double iguess = 0.0;
  int count = 0;
  double id;
  while (true) {
    count++;

    // Voltage across diode is the voltage across the terminals plus the voltage
    // across the series resistance
    double vd = vload + iguess * rs;

    // Current through rp is determined by the voltage across the diode
    double ip = vd / rp;

    // Get the diode current (return current)
    id = PvCell::dio_iret(vd, is1, n1);

    if (id > iph) {
      spdlog::warn("id > iph, open circuit voltage reached?. vload={:e}",
                   vload);
      return 0.0;
    } else if (count > 1e6) {
      spdlog::error("Couldn't converge after a million iterations. vload={:e}, "
                    "iguess={:e} after {:d} iterations",
                    vload, iguess, count);
      return 0.0;
    } else if (iguess < iph - id - ip - 5.0e-8) {
      // Continue guessing
      iguess += 1.0e-7;
    } else {
      /*
      spdlog::info(
          "Result converged at vload={:e}, iguess={:e} after {:d} iterations",
          vload, iguess, count);
          */
      break;
    }
  }

  return iguess;
}
