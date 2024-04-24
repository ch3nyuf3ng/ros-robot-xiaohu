#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_MEASUREMENT_HPP
#define XIAOHU_ROBOT_FOUNDATION_MEASUREMENT_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include <istream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace xiaohu_robot {
inline namespace Foundation {
class Unit: public Equatable<Unit>, public Printable {
public:
    virtual ~Unit() = default;

    virtual std::string getSymbol() const = 0;
    virtual void setSymbol(std::string symbol) = 0;
};

class UnitConverter: public Equatable<UnitConverter> {
public:
    virtual ~UnitConverter() = default;

    virtual double baseUnitValue(double fromValue) const = 0;
    virtual double value(double fromBaseUnitValue) const = 0;
};

class UnitConverterLinear final: public UnitConverter {
public:
    UnitConverterLinear(double coefficient, double constant);

    double getCoefficient() const;
    void setCoefficient(double coefficient);
    double getConstant() const;
    void setConstant(double constant);
    double baseUnitValue(double fromValue) const override;
    double value(double fromBaseUnitValue) const override;
    bool equals(UnitConverter const& object) const override;

private:
    double coefficient;
    double constant;
};

class ConvertibleUnit: public Unit, public std::enable_shared_from_this<ConvertibleUnit> {
public:
    virtual ~ConvertibleUnit() = default;

    virtual ConvertibleUnit const* getBaseUnit() const = 0;
    virtual UnitConverter const* getConverter() const = 0;
};

class TrivivalUnit final: public Unit {
public:
    TrivivalUnit(std::string symbol);
    bool equals(Unit const& object) const override;
    std::string getSymbol() const override;
    void setSymbol(std::string symbol) override;
    std::string toString() const override;

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
    makeVar(std::string symbol, double coefficient_to_base, double constant_to_base = 0) {
        return std::make_shared<UnitType>(ConstructorProtecter{}, symbol, coefficient_to_base, constant_to_base);
    }

    static std::shared_ptr<UnitType const>
    makeConst(std::string symbol, double coefficient_to_base, double constant_to_base = 0) {
        return std::make_shared<UnitType const>(ConstructorProtecter{}, symbol, coefficient_to_base, constant_to_base);
    }

    std::string getSymbol() const override final {
        return symbol;
    }

    void setSymbol(std::string symbol) override final {
        this->symbol = std::move(symbol);
    }

    UnitConverterLinear const* getConverter() const override final {
        return converter.get();
    }

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

    std::string toString() const override final {
        return symbol;
    }
};

class UnitLength final: public LinearConvertibleUnitBase<UnitLength> {
public:
    static std::shared_ptr<UnitLength const> const meters;
    static std::shared_ptr<UnitLength const> const kilometers;
    static std::shared_ptr<UnitLength const> const centimeters;

    using LinearConvertibleUnitBase::LinearConvertibleUnitBase;

    UnitLength const* getBaseUnit() const override;
};

class UnitDuration final: public LinearConvertibleUnitBase<UnitDuration> {
public:
    static std::shared_ptr<UnitDuration const> const milliseconds;
    static std::shared_ptr<UnitDuration const> const seconds;
    static std::shared_ptr<UnitDuration const> const minutes;
    static std::shared_ptr<UnitDuration const> const hours;

    using LinearConvertibleUnitBase<UnitDuration>::LinearConvertibleUnitBase;

    UnitDuration const* getBaseUnit() const override;
};

class UnitAngle final: public LinearConvertibleUnitBase<UnitAngle> {
public:
    static std::shared_ptr<UnitAngle const> const degrees;
    static std::shared_ptr<UnitAngle const> const arcMinutes;
    static std::shared_ptr<UnitAngle const> const arcSeconds;
    static std::shared_ptr<UnitAngle const> const radians;

    using LinearConvertibleUnitBase<UnitAngle>::LinearConvertibleUnitBase;

    UnitAngle const* getBaseUnit() const override;
};

class UnitSpeed final: public LinearConvertibleUnitBase<UnitSpeed> {
public:
    static std::shared_ptr<UnitSpeed const> const metersPerSecond;
    static std::shared_ptr<UnitSpeed const> const centimetersPerSecond;
    static std::shared_ptr<UnitSpeed const> const kilometersPerHour;

    using LinearConvertibleUnitBase<UnitSpeed>::LinearConvertibleUnitBase;

    UnitSpeed const* getBaseUnit() const override;
};

class UnitAngularSpeed final: public LinearConvertibleUnitBase<UnitAngularSpeed> {
public:
    static std::shared_ptr<UnitAngularSpeed const> const degreesPerSecond;
    static std::shared_ptr<UnitAngularSpeed const> const arcMinutesPerSecond;
    static std::shared_ptr<UnitAngularSpeed const> const arcSecondsPerSecond;
    static std::shared_ptr<UnitAngularSpeed const> const radiansPerSecond;

    using LinearConvertibleUnitBase<UnitAngularSpeed>::LinearConvertibleUnitBase;

    UnitAngularSpeed const* getBaseUnit() const override;
};

class UnitFrequency final: public LinearConvertibleUnitBase<UnitFrequency> {
public:
    static std::shared_ptr<UnitFrequency const> const hertz;
    static std::shared_ptr<UnitFrequency const> const kilohertz;

    using LinearConvertibleUnitBase<UnitFrequency>::LinearConvertibleUnitBase;

    UnitFrequency const* getBaseUnit() const override;
};

template<typename UnitType, typename = std::enable_if_t<std::is_base_of<Unit, UnitType>::value>>
struct Measurement final: public Equatable<Measurement<UnitType>>, public Printable {
private:
    double value;
    std::shared_ptr<UnitType const> unitSharedPtr;

public:
    explicit Measurement(double value, std::shared_ptr<UnitType const> unitSharedPtr):
        value{value},
        unitSharedPtr{std::move(unitSharedPtr)} {}

    double get_value() const {
        return value;
    }

    void set_value(double value) {
        this->value = std::move(value);
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    void convert(std::shared_ptr<UnitType const> other_unitSharedPtr) {
        auto otherUnitPtr{static_cast<ConvertibleUnit const*>(other_unitSharedPtr.get())};
        value = otherUnitPtr->getConverter()->value(getBaseUnitValue());
        unitSharedPtr = std::move(other_unitSharedPtr);
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    Measurement converted(std::shared_ptr<UnitType const> other_unitSharedPtr) const {
        auto otherUnitPtr{static_cast<ConvertibleUnit const*>(other_unitSharedPtr.get())};
        double converted_value{otherUnitPtr->getConverter()->value(getBaseUnitValue())};
        return Measurement{converted_value, std::move(other_unitSharedPtr)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    operator double() const {
        return getBaseUnitValue();
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    operator double() const {
        return value;
    }

    Measurement operator+() const {
        return *this;
    }

    Measurement operator-() const {
        return Measurement{-value, unitSharedPtr};
    }

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
        if (unitEquals(*this, rhs))
            value += rhs.value;
        else
            throw std::runtime_error{unitDoNotMatchInfo(*this, rhs)};
        return *this;
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    Measurement& operator+=(Measurement const& rhs) {
        if (unitEquals(*this, rhs))
            value += rhs.value;
        else
            value += getConverter()->value(rhs.getBaseUnitValue());
        return *this;
    }

    Measurement& operator-=(Measurement const& rhs) {
        return operator+=(-rhs);
    }

    Measurement& operator*=(double rhs) {
        value *= rhs;
        return *this;
    }

    Measurement& operator/=(double rhs) {
        value /= rhs;
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& ostr, const Measurement<UnitType>& m) {
        ostr << m.toString();
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
        if (unitEquals(lhs, rhs))
            return lhs.value / rhs.value;
        else
            throw std::runtime_error{unitDoNotMatchInfo(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend double operator/(Measurement const& lhs, Measurement const& rhs) {
        if (unitEquals(lhs, rhs))
            return lhs.value / rhs.value;
        else
            return lhs.getBaseUnitValue() / rhs.getBaseUnitValue();
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator<(Measurement const& lhs, Measurement const& rhs) {
        if (unitEquals(lhs, rhs))
            return lhs.value < rhs.value;
        else
            throw std::runtime_error{unitDoNotMatchInfo(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator<(Measurement const& lhs, Measurement const& rhs) {
        if (unitEquals(lhs, rhs))
            return lhs.value < rhs.value;
        else
            return lhs.getBaseUnitValue() < rhs.getBaseUnitValue();
    }

    friend bool operator>(Measurement const& lhs, Measurement const& rhs) {
        return rhs < lhs;
    }

    friend bool operator<=(Measurement const& lhs, Measurement const& rhs) {
        return !(lhs > rhs);
    }

    friend bool operator>=(Measurement const& lhs, Measurement const& rhs) {
        return !(lhs < rhs);
    }

    template<typename T = UnitType, std::enable_if_t<!std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator==(Measurement const& lhs, Measurement const& rhs) {
        if (unitEquals(lhs, rhs))
            return lhs.value == rhs.value;
        else
            throw std::runtime_error{unitDoNotMatchInfo(lhs, rhs)};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value, bool> = true>
    friend bool operator==(Measurement const& lhs, Measurement const& rhs) {
        if (unitEquals(lhs, rhs))
            return lhs.value == rhs.value;
        else
            return lhs.getBaseUnitValue() == rhs.getBaseUnitValue();
    }

    friend bool operator!=(Measurement const& lhs, Measurement const& rhs) {
        return !(lhs == rhs);
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, T>::value>>
    double getBaseUnitValue() const {
        return getConverter()->baseUnitValue(value);
    }

    bool equals(Measurement const& other) const override {
        return *this == other;
    }

    std::string toString() const override {
        std::ostringstream oss;
        oss << value << ' ' << getUnit()->getSymbol();
        return oss.str();
    }

    template<typename T = UnitType, std::enable_if_t<std::is_same<T, UnitFrequency>::value, bool> = true>
    Measurement<UnitDuration> perCycleTime() const {
        return Measurement<UnitDuration>{1 / getBaseUnitValue(), UnitDuration::seconds};
    }

    template<typename T = UnitType, std::enable_if_t<std::is_same<T, UnitDuration>::value, bool> = true>
    Measurement<UnitFrequency> accordingFrequency() const {
        return Measurement<UnitFrequency>{
            1 / getBaseUnitValue(), UnitFrequency::getBaseUnit()->shared_from_this()
        };
    }

private:
    static std::string unitDoNotMatchInfo(Measurement const& lhs, Measurement const& rhs) {
        auto oss{std::ostringstream{}};
        oss << "Units do not match and they're not convertible.\n";
        oss << "lhs unit: " << static_cast<Unit const*>(lhs.unitSharedPtr.get())->getSymbol() << '\n';
        oss << "rhs unit: " << static_cast<Unit const*>(rhs.unitSharedPtr.get())->getSymbol();
        return oss.str();
    }

    static bool unitEquals(Measurement const& lhs, Measurement const& rhs) {
        auto lhs_unit_ptr{static_cast<Unit const*>(lhs.unitSharedPtr.get())};
        auto rhs_unit_ptr{static_cast<Unit const*>(rhs.unitSharedPtr.get())};
        return lhs_unit_ptr->equals(*rhs_unit_ptr);
    }

    Unit const* getUnit() const {
        return unitSharedPtr.get();
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, UnitType>::value>>
    ConvertibleUnit const* getConvertibleUnit() const {
        return static_cast<ConvertibleUnit const*>(getUnit());
    }

    template<typename T = UnitType, typename = std::enable_if_t<std::is_base_of<ConvertibleUnit, UnitType>::value>>
    UnitConverter const* getConverter() const {
        return getConvertibleUnit()->getConverter();
    }
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
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arcMinutes};
}

inline Measurement<UnitAngle> operator""_arcmin(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arcMinutes};
}

inline Measurement<UnitAngle> operator""_arcsec(unsigned long long v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arcSeconds};
}

inline Measurement<UnitAngle> operator""_arcsec(long double v) {
    return Measurement<UnitAngle>{static_cast<double>(v), UnitAngle::arcSeconds};
}

inline Measurement<UnitSpeed> operator""_m_per_s(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::metersPerSecond};
}

inline Measurement<UnitSpeed> operator""_m_per_s(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::metersPerSecond};
}

inline Measurement<UnitSpeed> operator""_cm_per_s(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::centimetersPerSecond};
}

inline Measurement<UnitSpeed> operator""_cm_per_s(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::centimetersPerSecond};
}

inline Measurement<UnitSpeed> operator""_km_per_h(unsigned long long v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::kilometersPerHour};
}

inline Measurement<UnitSpeed> operator""_km_per_h(long double v) {
    return Measurement<UnitSpeed>{static_cast<double>(v), UnitSpeed::kilometersPerHour};
}

inline Measurement<UnitAngularSpeed> operator""_deg_per_s(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::degreesPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_deg_per_s(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::degreesPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_rad_per_s(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::radiansPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_rad_per_s(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::radiansPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_arcmin_per_s(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arcMinutesPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_arcmin_per_s(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arcMinutesPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_arcsec_per_s(unsigned long long v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arcSecondsPerSecond};
}

inline Measurement<UnitAngularSpeed> operator""_arcsec_per_s(long double v) {
    return Measurement<UnitAngularSpeed>{static_cast<double>(v), UnitAngularSpeed::arcSecondsPerSecond};
}

inline Measurement<UnitFrequency> operator""_Hz(long double v) {
    return Measurement<UnitFrequency>{static_cast<double>(v), UnitFrequency::hertz};
}

inline Measurement<UnitFrequency> operator""_Hz(unsigned long long v) {
    return Measurement<UnitFrequency>{static_cast<double>(v), UnitFrequency::hertz};
}

inline Measurement<UnitFrequency> operator""_kHz(long double v) {
    return Measurement<UnitFrequency>{static_cast<double>(v), UnitFrequency::kilohertz};
}

inline Measurement<UnitFrequency> operator""_kHz(unsigned long long v) {
    return Measurement<UnitFrequency>{static_cast<double>(v), UnitFrequency::kilohertz};
}
}  // namespace literals
}  // namespace Foundation
}  // namespace xiaohu_robot
#endif