#include "xiaohu_robot/Foundation/Measurement.hpp"

using namespace xiaohu_robot;
using std::shared_ptr;

/* class UnitConverterLinear */
UnitConverterLinear::UnitConverterLinear(double coefficient, double constant = 0):
    coefficient{coefficient},
    constant{constant} {}

double UnitConverterLinear::getCoefficient() const {
    return coefficient;
}

void UnitConverterLinear::setCoefficient(double coefficient) {
    this->coefficient = std::move(coefficient);
}

double UnitConverterLinear::getConstant() const {
    return coefficient;
}

void UnitConverterLinear::setConstant(double constant) {
    this->constant = std::move(constant);
}

double UnitConverterLinear::baseUnitValue(double from_value) const {
    return coefficient * from_value + constant;
}

double UnitConverterLinear::value(double from_base_unit_value) const {
    return (from_base_unit_value - constant) / coefficient;
}

bool UnitConverterLinear::equals(UnitConverter const& object) const {
    if (this == &object) {
        return true;
    }
    try {
        auto& other{dynamic_cast<UnitConverterLinear const&>(object)};
        return other.coefficient == coefficient && other.constant == constant;
    } catch (std::bad_cast const&) {
        return false;
    }
}

/* class TrivivalUnit */
TrivivalUnit::TrivivalUnit(std::string symbol):
    symbol{std::move(symbol)} {}

bool TrivivalUnit::equals(Unit const& object) const {
    if (&object == this) {
        return true;
    }
    try {
        auto& other = dynamic_cast<TrivivalUnit const&>(object);
        return other.symbol == symbol;
    } catch (std::bad_cast const& e) {
        return false;
    }
}

std::string TrivivalUnit::getSymbol() const {
    return symbol;
}

void TrivivalUnit::setSymbol(std::string symbol) {
    this->symbol = std::move(symbol);
}

std::string TrivivalUnit::toString() const {
    return symbol;
}

/* class UnitLength */
shared_ptr<UnitLength const> const UnitLength::meters{UnitLength::makeConst("m", 1)};
shared_ptr<UnitLength const> const UnitLength::kilometers{UnitLength::makeConst("km", 1000)};
shared_ptr<UnitLength const> const UnitLength::centimeters{UnitLength::makeConst("cm", 0.01)};

UnitLength const* UnitLength::getBaseUnit() const {
    return meters.get();
}

/* class UnitDuration */
shared_ptr<UnitDuration const> const UnitDuration::microseconds{UnitDuration::makeConst("us", 1e-6)};
shared_ptr<UnitDuration const> const UnitDuration::milliseconds{UnitDuration::makeConst("ms", 0.001)};
shared_ptr<UnitDuration const> const UnitDuration::seconds{UnitDuration::makeConst("s", 1)};
shared_ptr<UnitDuration const> const UnitDuration::minutes{UnitDuration::makeConst("min", 60)};
shared_ptr<UnitDuration const> const UnitDuration::hours{UnitDuration::makeConst("h", 3600)};

UnitDuration const* UnitDuration::getBaseUnit() const {
    return seconds.get();
}

/* class UnitAngle */
shared_ptr<UnitAngle const> const UnitAngle::degrees{UnitAngle::makeConst("deg", 1)};
shared_ptr<UnitAngle const> const UnitAngle::arcMinutes{UnitAngle::makeConst("arcmin", 0.0166667)};
shared_ptr<UnitAngle const> const UnitAngle::arcSeconds{UnitAngle::makeConst("arcsec", 0.00027778)};
shared_ptr<UnitAngle const> const UnitAngle::radians{UnitAngle::makeConst("rad", 57.2958)};

UnitAngle const* UnitAngle::getBaseUnit() const {
    return degrees.get();
}

/* class UnitSpeed */
shared_ptr<UnitSpeed const> const UnitSpeed::metersPerSecond{UnitSpeed::makeConst("m/s", 1)};
shared_ptr<UnitSpeed const> const UnitSpeed::centimetersPerSecond{UnitSpeed::makeConst("cm/s", 0.01)};
shared_ptr<UnitSpeed const> const UnitSpeed::kilometersPerHour{UnitSpeed::makeConst("km/h", 0.277778)};

UnitSpeed const* UnitSpeed::getBaseUnit() const {
    return metersPerSecond.get();
}

/* class UnitAngularSpeed */
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::degreesPerSecond{UnitAngularSpeed::makeConst("deg/s", 1)};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::arcMinutesPerSecond{
    UnitAngularSpeed::makeConst("arcmin/s", 0.0166667)
};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::arcSecondsPerSecond{
    UnitAngularSpeed::makeConst("arcsec/s", 0.00027778)
};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::radiansPerSecond{
    UnitAngularSpeed::makeConst("rad/s", 57.2958)
};

UnitAngularSpeed const* UnitAngularSpeed::getBaseUnit() const {
    return degreesPerSecond.get();
}

/* class UnitFrequency */
shared_ptr<UnitFrequency const> const UnitFrequency::hertz{UnitFrequency::makeConst("Hz", 1)};
shared_ptr<UnitFrequency const> const UnitFrequency::kilohertz{UnitFrequency::makeConst("KHz", 1000)};

UnitFrequency const* UnitFrequency::getBaseUnit() const {
    return hertz.get();
}