#pragma once

#include "Transform2D.h"
#include "StaticTransform2D.h"
#include "Boundaries2D.h"

#include <vector>

class b2Body;
class b2World;

namespace KEngineBox2D {

	class Box2DWorld;

	class Box2DTransform : public KEngine2D::Transform
	{
	public:
		Box2DTransform();
		virtual ~Box2DTransform();

		void Init(Box2DWorld * world, KEngine2D::BoundingArea * collisionVolume, double mass, KEngine2D::StaticTransform const & currentTransform, KEngine2D::Point const & velocity, double angularVelocity);

		void Deinit();

		void Update(double fTime);

		virtual KEngine2D::Point GetTranslation() const override;
		virtual double GetRotation() const override;
		virtual double GetScale() const override;
		virtual const KEngine2D::Matrix& GetAsMatrix() const override;

	private:
		b2Body * mBody;
		KEngine2D::StaticTransform mCurrentTransform;

	};



	class Box2DWorld
	{
	public:
		void Init(const KEngine2D::Point& gravity = KEngine2D::Point::Origin());
		void Deinit();

		void Update(double timestep);

		void AddWall(const KEngine2D::Point& start, const KEngine2D::Point& end, const KEngine2D::Point& normal);

	private:
		b2World * mWorld;
		friend class Box2DTransform;
		std::vector<Box2DTransform *> mUpdateList;
	};
}