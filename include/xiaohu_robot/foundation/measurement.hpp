#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_MEASUREMENT_HPP
#define XIAOHU_ROBOT_FOUNDATION_MEASUREMENT_HPP
#include "xiaohu_robot/foundation/common_interfaces.hpp"
#include <istream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace xiaohu_robot {
inline namespace foundation {
inline namespace measurement {
class Unit: public Equatable<Unit>, public Printable {
public:
    virtual ~Unit() = default;

    virtual std::string get_symbol() const = 0;
    virtual void set_symbol(std::string symbol) = 0;
};

class UnitConverter: public Equatable<UnitConverter> {
public:
    virtual ~UnitConverter() = default;

    virtual double base_unit_value(double from_value) const = 0;
    virtual double value(double from_base_unit_value) const = 0;
};

class UnitConverterLinear final: public UnitConverter {
public:
    UnitConverterLinear(double coefficient, double constant);

    double get_coefficient() const;
    void set_coefficient(double coefficient);
    double get_constant() const;
    void set_constant(double constant);
    double base_unit_value(double from_value) const override;
    double value(double from_base_unit_value) const override;
    bool equals(UnitConverter const& object) const override;

private:
    double coefficient;
    double constant;
};

class ConvertibleUnit: public Unit, public std::enable_shared_from_this<ConvertibleUnit> {
public:
    virtual ~ConvertibleUnit() = default;

    virtual ConvertibleUnit const* get_base_unit() const = 0;
    virtual UnitConverter const* get_converter() const = 0;
};

class TrivivalUnit final: public Unit {
public:
    TrivivalUnit(std::string symbol);
    bool equals(Unit const& object) const override;
    std::string get_symbol() const override;
    void set_symbol(std::string symbol) override;
    std::string to_string() const override;

private:
    std::string symbol;
};

template<typename UnitType> class LinearConvertibleUnitBase: public ConvertibleUnit {
private:
    std::string symbol;
    std::unique_ptr<UnitConverterLinear const> converter;

protected:
    struct ConstructorProtecter {};

public:
    explicit LinearConvertibleUnitBase(
        ConstructorProtecter, std::string symbol, double coefficient_to_base, double constant_to_base = 0
    ):
        symbol{std::move(symbol)},
        converter{std::make_unique<UnitConverterLinear>(coefficient_to_base, constant_to_base)} {}

    virtual ~LinearConvertibleUnitBase() = default;

    static std::shared_ptr<UnitType>
    create(std::string symbol, double coefficient_to_base, double constant_to_base = 0) {
        return std::make_shared<UnitType>(ConstructorProtecter{}, symbol, coefficient_to_base, constant_to_base);
    }

    static std::shared_ptr<UnitType const>
    create_const(std::string symbol, double coefficient_to_base, double constant_to_base = 0) {
        return std::make_shared<UnitType const>(ConstructorProtecter{}, symbol, coefficient_to_base, constant_to_base);
    }

    std::string get_symbol() const override final { return symbol; }

    void set_symbol(std::string symbol) override final { this->symbol = std::move(symbol); }

    UnitConverterLinear const* get_converter() const override final { return converter.get(); }

    bool equals(Unit const& object) const override final {
        if (this == &object) {
            return true;
        }
        try {
            auto& other{dynamic_cast<LinearConvertibleUnitBase const&>(object)};
            return other.symbol == symbol && other.converter->equals(*converter);
        }
        catch (std::bad_cast const&) {
            return false;
        }
    }

    std::string to_string() const override final { return symbol; }
};

template<typename UnitType, typename = std::enable_if_t<std::is_base_of<Unit, UnitType>::value>>
struct Measurement final: public Equatable<Measurement<UnitType>>, public Printable {
private:
    double value;
    std::shared_ptr<UnitType const> unit_shared_ptr;

public:
    explicit Measurement(double value, std::shared_ptr<UnitType const> unit_shared_ptr):
        value{value}, unit_shared_ptr{std::move(unit_shared_ptr)} {}

    double get_value() const { return value; }

    void set_value(double value) { this->value = std::move(value); }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    void convert(std::shared_ptr<UnitType const> other_unit_shared_ptr) {
        auto other_unit_ptr{static_cast<ConvertibleUnit const*>(other_unit_shared_ptr.get())};
        value = other_unit_ptr->get_converter()->value(get_base_unit_value());
        unit_shared_ptr = std::move(other_unit_shared_ptr);
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    Measurement converted(std::shared_ptr<UnitType const> other_unit_shared_ptr) const {
        auto other_unit_ptr{static_cast<ConvertibleUnit const*>(other_unit_shared_ptr.get())};
        double converted_value{other_unit_ptr->get_converter()->value(get_base_unit_value())};
        return Measurement{converted_value, std::move(other_unit_shared_ptr)};
    }

    Measurement operator+() const { return *this; }

    Measurement operator-() const { return Measurement{-value, unit_shared_ptr}; }

    Measurement& operator++() {
        value += 1;
        return *this;
    }

    Measurement operator++(int) {
        auto old_measurement{*this};
        operator++();
        return old_measurement;
    }

    Measurement& operator--() {
        value -= 1;
        return *this;
    }

    Measurement operator--(int) {
        auto old_measurement{*this};
        operator--();
        return old_measurement;
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    Measurement& operator+=(Measurement const& rhs) {
        if (unit_equals(*this, rhs))
            value += rhs.value;
        else
            throw std::runtime_error{unit_do_not_match_info(*this, rhs)};
        return *this;
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    Measurement& operator+=(Measurement const& rhs) {
        if (unit_equals(*this, rhs))
            value += rhs.value;
        else
            value += get_converter()->value(rhs.get_base_unit_value());
        return *this;
    }

    Measurement& operator-=(Measurement const& rhs) { return operator+=(-rhs); }

    Measurement& operator*=(double rhs) {
        value *= rhs;
        return *this;
    }

    Measurement& operator/=(double rhs) {
        value /= rhs;
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& ostr, const Measurement<UnitType>& m) {
        ostr << m.to_string();
        return ostr;
    }

    friend std::istream& operator>>(std::istream& istr, const Measurement<UnitType>& m) {
        istr >> m.value;
        return istr;
    }

    friend Measurement operator+(Measurement lhs, Measurement const& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend Measurement operator-(Measurement lhs, Measurement const& rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend Measurement operator*(Measurement lhs, double rhs) {
        lhs *= rhs;
        return lhs;
    }

    friend Measurement operator/(Measurement lhs, double rhs) {
        lhs /= rhs;
        return lhs;
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend double operator/(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value / rhs.value;
        else
            throw std::runtime_error{unit_do_not_match_info(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend double operator/(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value / rhs.value;
        else
            return lhs.get_base_unit_value() / rhs.get_base_unit_value();
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator<(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value < rhs.value;
        else
            throw std::runtime_error{unit_do_not_match_info(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator<(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value < rhs.value;
        else
            return lhs.get_base_unit_value() < rhs.get_base_unit_value();
    }

    friend bool operator>(Measurement const& lhs, Measurement const& rhs) { return rhs < lhs; }

    friend bool operator<=(Measurement const& lhs, Measurement const& rhs) { return !(lhs > rhs); }

    friend bool operator>=(Measurement const& lhs, Measurement const& rhs) { return !(lhs < rhs); }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator==(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value == rhs.value;
        else
            throw std::runtime_error{unit_do_not_match_info(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator==(Measurement const& lhs, Measurement const& rhs) {
        if (unit_equals(lhs, rhs))
            return lhs.value == rhs.value;
        else
            return lhs.get_base_unit_value() == rhs.get_base_unit_value();
    }

    friend bool operator!=(Measurement const& lhs, Measurement const& rhs) { return !(lhs == rhs); }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    double get_base_unit_value() const {
        return get_converter()->base_unit_value(value);
    }

    bool equals(Measurement const& other) const override { return *this == other; }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << value << ' ' << get_unit()->get_symbol();
        return oss.str();
    }

private:
    static std::string unit_do_not_match_info(Measurement const& lhs, Measurement const& rhs) {
        auto oss{std::ostringstream{}};
        oss << "Units do not match and they're not convertible.\n";
        oss << "lhs unit: " << static_cast<Unit const*>(lhs.unit_shared_ptr.get())->get_symbol() << '\n';
        oss << "rhs unit: " << static_cast<Unit const*>(rhs.unit_shared_ptr.get())->get_symbol();
        return oss.str();
    }

    static bool unit_equals(Measurement const& lhs, Measurement const& rhs) {
        auto lhs_unit_ptr{static_cast<Unit const*>(lhs.unit_shared_ptr.get())};
        auto rhs_unit_ptr{static_cast<Unit const*>(rhs.unit_shared_ptr.get())};
        return lhs_unit_ptr->equals(*rhs_unit_ptr);
    }

    Unit const* get_unit() const { return unit_shared_ptr.get(); }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, UnitType>::value>>
    ConvertibleUnit const* get_convertible_unit() const {
        return static_cast<ConvertibleUnit const*>(get_unit());
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, UnitType>::value>>
    UnitConverter const* get_converter() const {
        return get_convertible_unit()->get_converter();
    }
};

class UnitLength final: public LinearConvertibleUnitBase<UnitLength> {
public:
    static std::shared_ptr<UnitLength const> const meters;
    static std::shared_ptr<UnitLength const> const kilometers;
    static std::shared_ptr<UnitLength const> const centimeters;

    using LinearConvertibleUnitBase::LinearConvertibleUnitBase;

    UnitLength const* get_base_unit() const override;
};

class UnitDuration final: public LinearConvertibleUnitBase<UnitDuration> {
public:
    static std::shared_ptr<UnitDuration const> const milliseconds;
    static std::shared_ptr<UnitDuration const> const seconds;
    static std::shared_ptr<UnitDuration const> const minutes;
    static std::shared_ptr<UnitDuration const> const hours;

    using LinearConvertibleUnitBase<UnitDuration>::LinearConvertibleUnitBase;

    UnitDuration const* get_base_unit() const override;
};

class UnitAngle final: public LinearConvertibleUnitBase<UnitAngle> {
public:
    static std::shared_ptr<UnitAngle const> const degrees;
    static std::shared_ptr<UnitAngle const> const arc_minutes;
    static std::shared_ptr<UnitAngle const> const arc_seconds;
    static std::shared_ptr<UnitAngle const> const radians;

    using LinearConvertibleUnitBase<UnitAngle>::LinearConvertibleUnitBase;

    UnitAngle const* get_base_unit() const override { return degrees.get(); }
};

class UnitSpeed final: public LinearConvertibleUnitBase<UnitSpeed> {
public:
    static std::shared_ptr<UnitSpeed const> const meters_per_second;
    static std::shared_ptr<UnitSpeed const> const centimeters_per_second;
    static std::shared_ptr<UnitSpeed const> const killometers_per_hour;

    using LinearConvertibleUnitBase<UnitSpeed>::LinearConvertibleUnitBase;

    UnitSpeed const* get_base_unit() const override;
};

class UnitAngularSpeed final: public LinearConvertibleUnitBase<UnitAngularSpeed> {
public:
    static std::shared_ptr<UnitAngularSpeed const> const degrees_per_second;
    static std::shared_ptr<UnitAngularSpeed const> const arc_minutes_per_second;
    static std::shared_ptr<UnitAngularSpeed const> const arc_seconds_per_second;
    static std::shared_ptr<UnitAngularSpeed const> const radians_per_second;

    using LinearConvertibleUnitBase<UnitAngularSpeed>::LinearConvertibleUnitBase;

    UnitAngularSpeed const* get_base_unit() const override;
};

inline namespace literals {
inline Measurement<UnitLength> operator""_cm(unsigned long long v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::centimeters};
}

inline Measurement<UnitLength> operator""_cm(long double v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::centimeters};
}

inline Measurement<UnitLength> operator""_m(unsigned long long v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::meters};
}

inline Measurement<UnitLength> operator""_m(long double v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::meters};
}

inline Measurement<UnitLength> operator""_km(unsigned long long v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::kilometers};
}

inline Measurement<UnitLength> operator""_km(long double v) {
    return Measurement<UnitLength>{static_cast<double>(v), UnitLength::kilometers};
}

inline Measurement<UnitDuration> operator""_s(unsigned long long v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::seconds};
}

inline Measurement<UnitDuration> operator""_s(long double v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::seconds};
}

inline Measurement<UnitDuration> operator""_ms(unsigned long long v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::milliseconds};
}

inline Measurement<UnitDuration> operator""_ms(long double v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::milliseconds};
}

inline Measurement<UnitDuration> operator""_min(unsigned long long v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::minutes};
}

inline Measurement<UnitDuration> operator""_min(long double v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::minutes};
}

inline Measurement<UnitDuration> operator""_h(unsigned long long v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::hours};
}

inline Measurement<UnitDuration> operator""_h(long double v) {
    return Measurement<UnitDuration>{static_cast<double>(v), UnitDuration::hours};
}

inline Measurement<UnitAngle> operator""_deg(unsigned long long v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::degrees};
}

inline Measurement<UnitAngle> operator""_deg(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::degrees};
}

inline Measurement<UnitAngle> operator""_rad(unsigned long long v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::radians};
}

inline Measurement<UnitAngle> operator""_rad(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::radians};
}

inline Measurement<UnitAngle> operator""_arcmin(unsigned long long v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arc_minutes};
}

inline Measurement<UnitAngle> operator""_arcmin(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arc_minutes};
}

inline Measurement<UnitAngle> operator""_arcsec(unsigned long long v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arc_seconds};
}

inline Measurement<UnitAngle> operator""_arcsec(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arc_seconds};
}

inline Measurement<UnitSpeed> operator""_mPs(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::meters_per_second};
}

inline Measurement<UnitSpeed> operator""_mPs(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::meters_per_second};
}

inline Measurement<UnitSpeed> operator""_cmPs(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::centimeters_per_second};
}

inline Measurement<UnitSpeed> operator""_cmPs(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::centimeters_per_second};
}

inline Measurement<UnitSpeed> operator""_kmPh(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::killometers_per_hour};
}

inline Measurement<UnitSpeed> operator""_kmPh(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::killometers_per_hour};
}

inline Measurement<UnitAngularSpeed> operator""_degPs(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::degrees_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_degPs(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::degrees_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_radPs(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::radians_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_radPs(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::radians_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_arcminPs(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arc_minutes_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_arcminPs(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arc_minutes_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_arcsecPs(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arc_seconds_per_second};
}

inline Measurement<UnitAngularSpeed> operator""_arcsecPs(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arc_seconds_per_second};
}
}  // namespace literals
}  // namespace measurement
}  // namespace foundation
}  // namespace xiaohu_robot
#endif