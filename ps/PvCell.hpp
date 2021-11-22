namespace PvCell {

/**
 * @brief Diode model
 */
double dio_iret(double v, double is1, double n);

/**
 * @brief PvCell model that consumes lux and load current to calculate output
 *        voltage
 * @param lux light intensity
 * @param iload load current
 * @retval Output voltage
 */
double pv(const double lux, const double iload);

/**
 * @briev PV cell model that consumes lux & load voltage to calculate output
 *        *current*
 */
double singleDiodeEquation(const double lux, const double vload);

} // namespace PvCell
