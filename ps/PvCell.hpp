namespace PvCell {

/**
 * @brief Diode model
 */
double dio_iret(double v, double is1, double n);

/**
 * @brief PvCell model
 * @param lux light intensity
 * @param iload load current
 * @retval Output voltage
 */
double pv(const double lux, const double iload);

/**
 * @brief diode reverse current for singleDiodeEquation.
 */
double dio_iret_v2(double v, double i, double rs, double is1, double n);

/**
 * @briev PV cell model that consumes lux & load voltage to calculate output
 * current
 */
double singleDiodeEquation(const double lux, const double vload);

} // namespace PvCell
