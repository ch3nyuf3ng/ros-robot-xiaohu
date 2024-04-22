#include "xiaohu/foundation/measurement.hpp"

using namespace xiaohu_robot;
using std::shared_ptr;

/* class UnitConverterLinear */
UnitConverterLinear::UnitConverterLinear(double coefficient, double constant = 0):
    coefficient{coefficient}, constant{constant} {}

double UnitConverterLinear::get_coefficient() const {
    return coefficient;
}

void UnitConverterLinear::set_coefficient(double coefficient) {
    this->coefficient = std::move(coefficient);
}

double UnitConverterLinear::get_constant() const {
    return coefficient;
}

void UnitConverterLinear::set_constant(double constant) {
    this->constant = std::move(constant);
}

double UnitConverterLinear::base_unit_value(double from_value) const {
    return coefficient * from_value + constant;
}

double UnitConverterLinear::value(double from_base_unit_value) const {
    return (from_base_unit_value - constant) / coefficient;
}

bool UnitConverterLinear::equals(UnitConverter const& object) const {
    if (this == &object)
        return true;
    try {
        auto& other{dynamic_cast<UnitConverterLinear const&>(object)};
        return other.coefficient == coefficient && other.constant == constant;
    }
    catch (std::bad_cast const&) {
        return false;
    }
}

/* class TrivivalUnit */
TrivivalUnit::TrivivalUnit(std::string symbol): symbol{std::move(symbol)} {}

bool TrivivalUnit::equals(Unit const& object) const {
    if (&object == this) {
        return true;
    }
    try {
        auto& other = dynamic_cast<TrivivalUnit const&>(object);
        return other.symbol == symbol;
    }
    catch (std::bad_cast const& e) {
        return false;
    }
}

std::string TrivivalUnit::get_symbol() const {
    return symbol;
}

void TrivivalUnit::set_symbol(std::string symbol) {
    this->symbol = std::move(symbol);
}

std::string TrivivalUnit::to_string() const {
    return symbol;
}

/* class UnitLength */
shared_ptr<UnitLength const> const UnitLength::meters{UnitLength::create_const("m", 1)};
shared_ptr<UnitLength const> const UnitLength::kilometers{UnitLength::create_const("km", 1000)};
shared_ptr<UnitLength const> const UnitLength::centimeters{UnitLength::create_const("cm", 0.01)};

UnitLength const* UnitLength::get_base_unit() const {
    return meters.get();
}

/* class UnitDuration */
shared_ptr<UnitDuration const> const UnitDuration::milliseconds{UnitDuration::create_const("ms", 0.001)};
shared_ptr<UnitDuration const> const UnitDuration::seconds{UnitDuration::create_const("s", 1)};
shared_ptr<UnitDuration const> const UnitDuration::minutes{UnitDuration::create_const("min", 60)};
shared_ptr<UnitDuration const> const UnitDuration::hours{UnitDuration::create_const("h", 3600)};

UnitDuration const* UnitDuration::get_base_unit() const {
    return seconds.get();
}

/* class UnitAngle */
shared_ptr<UnitAngle const> const UnitAngle::degrees{UnitAngle::create_const("°", 1)};
shared_ptr<UnitAngle const> const UnitAngle::arc_minutes{UnitAngle::create_const("'", 0.0166667)};
shared_ptr<UnitAngle const> const UnitAngle::arc_seconds{UnitAngle::create_const("\"", 0.00027778)};
shared_ptr<UnitAngle const> const UnitAngle::radians{UnitAngle::create_const("rad", 57.2958)};

UnitSpeed const* UnitSpeed::get_base_unit() const {
    return meters_per_second.get();
}

/* class UnitSpeed */
shared_ptr<UnitSpeed const> const UnitSpeed::meters_per_second{UnitSpeed::create_const("m/s", 1)};
shared_ptr<UnitSpeed const> const UnitSpeed::centimeters_per_second{UnitSpeed::create_const("cm/s", 0.01)};
shared_ptr<UnitSpeed const> const UnitSpeed::killometers_per_hour{UnitSpeed::create_const("km/h", 0.277778)};

UnitAngularSpeed const* UnitAngularSpeed::get_base_unit() const {
    return degrees_per_second.get();
}

/* class UnitAngularSpeed */
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::degrees_per_second{UnitAngularSpeed::create_const("°/s", 1)};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::arc_minutes_per_second{
    UnitAngularSpeed::create_const("'/s", 0.0166667)
};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::arc_seconds_per_second{
    UnitAngularSpeed::create_const("\"/s", 0.00027778)
};
shared_ptr<UnitAngularSpeed const> const UnitAngularSpeed::radians_per_second{
    UnitAngularSpeed::create_const("rad/s", 57.2958)
};
