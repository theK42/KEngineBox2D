#include "Box2DTransform.h"
#include "Box2D/Box2D.h"


KEngineBox2D::Box2DTransform::Box2DTransform()
{
}


KEngineBox2D::Box2DTransform::~Box2DTransform()
{
}

void KEngineBox2D::Box2DTransform::Init(Box2DWorld * world, KEngine2D::BoundingArea * collisionVolume, double mass, KEngine2D::StaticTransform const & currentTransform, KEngine2D::Point const & velocity, double angularVelocity)
{
	b2BodyDef bodyDef;
	bodyDef.position.x = currentTransform.GetTranslation().x;
	bodyDef.position.y = currentTransform.GetTranslation().y;
	bodyDef.angle = currentTransform.GetRotation();
	bodyDef.linearVelocity.x = velocity.x;
	bodyDef.linearVelocity.y = velocity.y;
	bodyDef.type = b2_dynamicBody;
	mBody = world->mWorld->CreateBody(&bodyDef);

	int fixtureCount = collisionVolume->GetBoundingBoxes().size() + collisionVolume->GetBoundingCircles().size();
	double massPerFixture = mass / fixtureCount;

	for (const KEngine2D::BoundingBox* box : collisionVolume->GetBoundingBoxes()) {
		b2FixtureDef fixtureDef;
		b2PolygonShape rect;
		rect.SetAsBox(box->GetWidth() / 2.0f, box->GetHeight() / 2.0f);
		rect.m_centroid.x = box->GetCenter().x;
		rect.m_centroid.y = box->GetCenter().y;
		fixtureDef.shape = &rect;
		fixtureDef.density = massPerFixture / box->GetArea();
		fixtureDef.restitution = 1.0f;
		mBody->CreateFixture(&fixtureDef);
	}
	for (const KEngine2D::BoundingCircle* circle : collisionVolume->GetBoundingCircles()) {
		b2FixtureDef fixtureDef;
		b2CircleShape circ;
		circ.m_radius = circle->GetRadius();

		KEngine2D::Point relPos = circle->GetCenter();
		relPos -= currentTransform.GetTranslation();

		circ.m_p.x = relPos.x;
		circ.m_p.y = relPos.y;
		fixtureDef.shape = &circ;
		fixtureDef.density = massPerFixture / circle->GetArea();
		fixtureDef.restitution = 1.0f;
		mBody->CreateFixture(&fixtureDef);
	}
	world->mUpdateList.push_back(this);
}

void KEngineBox2D::Box2DTransform::Deinit()
{
	mBody->GetWorld()->DestroyBody(mBody);
	//TODO:  remove from update list
}

void KEngineBox2D::Box2DTransform::Update(double fTime)
{
	mCurrentTransform.SetTranslation({ mBody->GetPosition().x, mBody->GetPosition().y });
	mCurrentTransform.SetRotation(mBody->GetAngle());
}

KEngine2D::Point KEngineBox2D::Box2DTransform::GetTranslation() const
{
	return mCurrentTransform.GetTranslation();
}

double KEngineBox2D::Box2DTransform::GetRotation() const
{
	return mCurrentTransform.GetRotation();
}

double KEngineBox2D::Box2DTransform::GetScale() const
{
	return mCurrentTransform.GetScale();
}

const KEngine2D::Matrix & KEngineBox2D::Box2DTransform::GetAsMatrix() const
{
	return mCurrentTransform.GetAsMatrix();
}

void KEngineBox2D::Box2DWorld::Init(const KEngine2D::Point & gravity)
{
	b2Vec2 grav;
	grav.x = gravity.x;
	grav.y = gravity.y;

	mWorld = new b2World(grav);
}

void KEngineBox2D::Box2DWorld::Deinit()
{
	delete mWorld;
}

void KEngineBox2D::Box2DWorld::Update(double timestep)
{
	mWorld->Step(timestep, 5, 5);
	for (Box2DTransform * transform : mUpdateList) {
		transform->Update(timestep);
	}
}

void KEngineBox2D::Box2DWorld::AddWall(const KEngine2D::Point & start, const KEngine2D::Point& end, const KEngine2D::Point& normal)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	b2Body * body = mWorld->CreateBody(&bodyDef);
	
	b2FixtureDef fixtureDef;
	b2PolygonShape rect;
	b2Vec2 vecs[4];
	vecs[0] = b2Vec2(start.x, start.y);
	vecs[1] = b2Vec2(end.x, end.y);
	vecs[2] = b2Vec2(end.x + normal.x, end.y + normal.y);
	vecs[3] = b2Vec2(start.x + normal.x, start.y + normal.y);
	rect.Set(vecs, 4);
	fixtureDef.shape = &rect;
	body->CreateFixture(&fixtureDef);
}
