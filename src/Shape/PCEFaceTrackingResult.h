#ifndef _PCEFaceTrackingResult
#define _PCEFaceTrackingResult

#include "PCEMacros.h"
#include <vector>

class PCEFaceTrackingResult
{
public:
	PCEFaceTrackingResult()
		: m_pAUs(NULL)
		, m_auCount(0)
		, m_scale(1.0f)
	{
		m_rotationXYZ[0] = 0.0f;
		m_rotationXYZ[1] = 0.0f;
		m_rotationXYZ[2] = 0.0f;
		m_translationXYZ[0] = 0.0f;
		m_translationXYZ[1] = 0.0f;
		m_translationXYZ[2] = 0.0f;
	}
	~PCEFaceTrackingResult() { clear(); }

	PCEFaceTrackingResult (const PCEFaceTrackingResult& other)
	{
		deepCopy(other);
	}
	PCEFaceTrackingResult& operator = (const PCEFaceTrackingResult& other)
	{
		deepCopy(other);
		return *this;
	}

	void deepCopy (const PCEFaceTrackingResult& other)
	{
		float*	pAUs = allocateAUs(other.m_auCount);
		if(pAUs)
		{
			memcpy(pAUs, other.m_pAUs, (sizeof(float) * m_auCount));
		}

		m_pts3DArray = other.m_pts3DArray;
		m_triangles = other.m_triangles;
		m_centerPt = other.m_centerPt;
		m_scale = other.m_scale;
		m_rotationXYZ[0] = other.m_rotationXYZ[0];
		m_rotationXYZ[1] = other.m_rotationXYZ[1];
		m_rotationXYZ[2] = other.m_rotationXYZ[2];
		m_translationXYZ[0] = other.m_translationXYZ[0];
		m_translationXYZ[1] = other.m_translationXYZ[1];
		m_translationXYZ[2] = other.m_translationXYZ[2];
	}

	void clear()
	{
		if(m_pAUs)
		{
			_freea(m_pAUs);
			m_pAUs = NULL;
		}
		m_pts3DArray.clear();
		m_triangles.clear();
	}

	float* allocateAUs(unsigned int	auCount)
	{
		if(auCount != m_auCount)
		{
			if(m_pAUs)
				_freea(m_pAUs);
			m_pAUs = NULL;
		}
		m_auCount = auCount;
		if(!m_pAUs && m_auCount != 0)
			m_pAUs = reinterpret_cast<float*>(_malloca(sizeof(float) * m_auCount));
		return m_pAUs;
	}

	void writeAUs(float* pAUs, unsigned int auCount)
	{
		if(!pAUs || 0 == auCount)
			return;
		if(!m_pAUs || (auCount != m_auCount))
		{
			if(m_pAUs)
				_freea(m_pAUs);
			m_auCount = auCount;
			m_pAUs = reinterpret_cast<float*>(_malloca(sizeof(float) * m_auCount));
		}
		if(!m_pAUs)
			return;
		memcpy(m_pAUs, pAUs, (sizeof(float) * m_auCount));
	}

	float* getAUs() const { return m_pAUs; }
	unsigned int getAUcount() const { return m_auCount; }
	void setAUcount(unsigned int count) {  m_auCount = count; }

	bool isMeshEmpty() const { return (m_pts3DArray.empty() || m_triangles.empty()); }

	void pushbackPts3D(Vector3 pts) { m_pts3DArray.push_back(pts); }
	Vector3 getPts3D(int id) const { return m_pts3DArray.at(id); }
	size_t sizeOfPts3D() const { return m_pts3DArray.size(); }

	void pushbackTriangle(Triangle tri) { m_triangles.push_back(tri); }
	Triangle getTriangle(int id) const { return m_triangles.at(id); }
	size_t sizeOfTriangles() const { return m_triangles.size(); }
	
	void setCenterPt(Vector3 centerPt) { m_centerPt = centerPt; }
	const Vector3& getCenterPt() const { return m_centerPt; }

	void setScale(float scale) { m_scale = scale; }
	float getScale() const { return m_scale; }

	void setRotationXYZ(float* pRotationXYZ)
	{
		if(!pRotationXYZ)
			return;
		m_rotationXYZ[0] = pRotationXYZ[0];
		m_rotationXYZ[1] = pRotationXYZ[1];
		m_rotationXYZ[2] = pRotationXYZ[2];
	}
	void getRotationXYZ(float* pRotationXYZ) const
	{
		if(!pRotationXYZ)
			return;
		pRotationXYZ[0] = m_rotationXYZ[0];
		pRotationXYZ[1] = m_rotationXYZ[1];
		pRotationXYZ[2] = m_rotationXYZ[2];
	}

	void setTranslationXYZ(float* pTranslationXYZ)
	{
		if(!pTranslationXYZ)
			return;
		m_translationXYZ[0] = pTranslationXYZ[0];
		m_translationXYZ[1] = pTranslationXYZ[1];
		m_translationXYZ[2] = pTranslationXYZ[2];
	}
	void getTranslationXYZ(float* pTranslationXYZ) const
	{
		if(!pTranslationXYZ)
			return;
		pTranslationXYZ[0] = m_translationXYZ[0];
		pTranslationXYZ[1] = m_translationXYZ[1];
		pTranslationXYZ[2] = m_translationXYZ[2];
	}

private:
	float*					m_pAUs;
	unsigned int			m_auCount;
	std::vector<Vector3>	m_pts3DArray;
	std::vector<Triangle>	m_triangles;
	Vector3					m_centerPt;
	float					m_scale;
	float					m_rotationXYZ[3];
	float					m_translationXYZ[3];
};

#endif