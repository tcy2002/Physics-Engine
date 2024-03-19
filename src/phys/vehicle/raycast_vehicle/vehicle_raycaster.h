#pragma once

#include "phys/phys_general.h"

/// VehicleRaycaster is provides interface for between vehicle simulation and raycasting
struct VehicleRaycaster
{
	virtual ~VehicleRaycaster() {}
	struct VehicleRaycasterResult {
		VehicleRaycasterResult() : m_distFraction(pe::Real(-1.)){};
		pe::Vector3 m_hitPointInWorld;
		pe::Vector3 m_hitNormalInWorld;
		pe::Real m_distFraction;
	};

	virtual void* castRay(const pe::Vector3& from, const pe::Vector3& to, VehicleRaycasterResult& result) = 0;
};
