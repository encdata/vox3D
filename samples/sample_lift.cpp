#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"
#include "box3d/voxel.h"

#include <vector>

// Helper to create an axis-aligned box hull at an arbitrary local center offset
static b3HullData* CreateBoxHullOffset( b3Vec3 halfExtents, b3Vec3 center )
{
	b3Vec3 pts[8];
	int idx = 0;
	for ( float sx : { -1.0f, 1.0f } )
	{
		for ( float sy : { -1.0f, 1.0f } )
		{
			for ( float sz : { -1.0f, 1.0f } )
			{
				pts[idx++] = { center.x + sx * halfExtents.x, center.y + sy * halfExtents.y,
							   center.z + sz * halfExtents.z };
			}
		}
	}
	return b3CreateHull( pts, 8, 8 );
}

// Helper to create an airplane wing hull with span along X, chord along Z (nose at -Z, tail at +Z),
// thickness and camber along Y, and dihedral angle (wingtips elevated).
static b3HullData* CreateGliderWingHull( float span, float chord, float thicknessRatio, float camberRatio,
										float dihedralDeg, b3Vec3 center )
{
	enum
	{
		kProfilePoints = 10,
		kSpanStations = 6
	};

	float dihedralRad = dihedralDeg * B3_PI / 180.0f;
	std::vector<b3Vec3> allPoints;
	allPoints.reserve( kProfilePoints * 2 * kSpanStations );

	for ( int s = 0; s < kSpanStations; ++s )
	{
		float u = (float)s / (float)( kSpanStations - 1 );
		float x = ( u - 0.5f ) * span;
		float yDihedral = fabsf( x ) * sinf( dihedralRad );

		for ( int i = 0; i < kProfilePoints; ++i )
		{
			float t = (float)i / (float)( kProfilePoints - 1 );
			// Cosine spacing from leading edge (t=0, z=-chord/2) to trailing edge (t=1, z=+chord/2)
			float zFrac = 0.5f * ( 1.0f - cosf( t * B3_PI ) );
			float zLocal = ( zFrac - 0.5f ) * chord;

			float xNorm = zFrac;
			// NACA thickness profile
			float yt = 5.0f * thicknessRatio * chord *
					   ( 0.2969f * sqrtf( xNorm ) - 0.1260f * xNorm - 0.3516f * xNorm * xNorm +
						 0.2843f * xNorm * xNorm * xNorm - 0.1015f * xNorm * xNorm * xNorm * xNorm );
			// Camber line
			float yc = 4.0f * camberRatio * chord * xNorm * ( 1.0f - xNorm );

			float yUpper = center.y + yDihedral + yc + yt;
			float yLower = center.y + yDihedral + yc - yt;
			float zPos = center.z + zLocal;
			float xPos = center.x + x;

			allPoints.push_back( { xPos, yUpper, zPos } );
			allPoints.push_back( { xPos, yLower, zPos } );
		}
	}

	return b3CreateHull( allPoints.data(), (int)allPoints.size(), 32 );
}

// Helper to create a swept-back vertical stabilizer fin
static b3HullData* CreateSweptVerticalFinHull( float baseLength, float tipLength, float height, float sweepBack,
											  float thickness, b3Vec3 rootCenter )
{
	b3Vec3 pts[8];
	float halfT = 0.5f * thickness;

	float zBaseFront = rootCenter.z - 0.5f * baseLength;
	float zBaseRear = rootCenter.z + 0.5f * baseLength;
	float zTipFront = zBaseFront + sweepBack;
	float zTipRear = zTipFront + tipLength;
	float yBase = rootCenter.y;
	float yTip = rootCenter.y + height;

	int idx = 0;
	for ( float signX : { -1.0f, 1.0f } )
	{
		float x = rootCenter.x + signX * halfT;
		pts[idx++] = { x, yBase, zBaseFront };
		pts[idx++] = { x, yBase, zBaseRear };
		pts[idx++] = { x, yTip, zTipFront };
		pts[idx++] = { x, yTip, zTipRear };
	}

	return b3CreateHull( pts, 8, 8 );
}

// Helper to create a single swept delta fin for darts at the tail end
static b3HullData* CreateDartSingleFinHull( float xFront, float xRear, float rRoot, float rTip, float thickness,
											int orientation )
{
	b3Vec3 pts[6];
	float halfT = 0.5f * thickness;

	if ( orientation == 0 ) // Top fin (+Y)
	{
		pts[0] = { xFront, rRoot, -halfT };
		pts[1] = { xFront, rRoot, halfT };
		pts[2] = { xRear, rRoot, -halfT };
		pts[3] = { xRear, rRoot, halfT };
		pts[4] = { xRear, rTip, -halfT };
		pts[5] = { xRear, rTip, halfT };
	}
	else if ( orientation == 1 ) // Bottom fin (-Y)
	{
		pts[0] = { xFront, -rRoot, -halfT };
		pts[1] = { xFront, -rRoot, halfT };
		pts[2] = { xRear, -rRoot, -halfT };
		pts[3] = { xRear, -rRoot, halfT };
		pts[4] = { xRear, -rTip, -halfT };
		pts[5] = { xRear, -rTip, halfT };
	}
	else if ( orientation == 2 ) // Right fin (+Z)
	{
		pts[0] = { xFront, -halfT, rRoot };
		pts[1] = { xFront, halfT, rRoot };
		pts[2] = { xRear, -halfT, rRoot };
		pts[3] = { xRear, halfT, rRoot };
		pts[4] = { xRear, -halfT, rTip };
		pts[5] = { xRear, halfT, rTip };
	}
	else // Left fin (-Z)
	{
		pts[0] = { xFront, -halfT, -rRoot };
		pts[1] = { xFront, halfT, -rRoot };
		pts[2] = { xRear, -halfT, -rRoot };
		pts[3] = { xRear, halfT, -rRoot };
		pts[4] = { xRear, -halfT, -rTip };
		pts[5] = { xRear, halfT, -rTip };
	}

	return b3CreateHull( pts, 6, 16 );
}

// Helper to create a single radial pitched rotor blade in the hub local space
static b3HullData* CreateRadialRotorBladeHull( float innerRadius, float outerRadius, float chord, float pitchDeg,
											  float azimuthRad, bool flipRotation = false )
{
	enum
	{
		kRadialStations = 5,
		kChordPoints = 6
	};

	float pitchRad = pitchDeg * B3_PI / 180.0f;
	float sinA = sinf( azimuthRad );
	float cosA = cosf( azimuthRad );

	// Radial direction
	b3Vec3 uR = { sinA, 0.0f, cosA };
	// Chord / tangent direction (in direction of rotation around +Y, or inverted for CCW)
	b3Vec3 uC = flipRotation ? b3Vec3{ -cosA, 0.0f, sinA } : b3Vec3{ cosA, 0.0f, -sinA };
	// Up direction
	b3Vec3 uY = { 0.0f, 1.0f, 0.0f };

	// Pitched chord and thickness directions
	b3Vec3 vChord = { cosf( pitchRad ) * uC.x, sinf( pitchRad ), cosf( pitchRad ) * uC.z };
	b3Vec3 vThick = { -sinf( pitchRad ) * uC.x, cosf( pitchRad ), -sinf( pitchRad ) * uC.z };

	std::vector<b3Vec3> pts;
	pts.reserve( kRadialStations * kChordPoints * 2 );

	for ( int rs = 0; rs < kRadialStations; ++rs )
	{
		float u = (float)rs / (float)( kRadialStations - 1 );
		float r = innerRadius + u * ( outerRadius - innerRadius );
		b3Vec3 rootP = { r * uR.x, 0.0f, r * uR.z };

		for ( int cp = 0; cp < kChordPoints; ++cp )
		{
			float t = (float)cp / (float)( kChordPoints - 1 );
			float s = ( t - 0.5f ) * chord;
			float xNorm = t;

			// Thickness
			float yt = 5.0f * 0.10f * chord *
					   ( 0.2969f * sqrtf( xNorm ) - 0.1260f * xNorm - 0.3516f * xNorm * xNorm +
						 0.2843f * xNorm * xNorm * xNorm - 0.1015f * xNorm * xNorm * xNorm * xNorm );
			float yc = 4.0f * 0.02f * chord * xNorm * ( 1.0f - xNorm );

			float upper = yc + yt;
			float lower = yc - yt;

			b3Vec3 pUpper = { rootP.x + s * vChord.x + upper * vThick.x,
							  rootP.y + s * vChord.y + upper * vThick.y,
							  rootP.z + s * vChord.z + upper * vThick.z };
			b3Vec3 pLower = { rootP.x + s * vChord.x + lower * vThick.x,
							  rootP.y + s * vChord.y + lower * vThick.y,
							  rootP.z + s * vChord.z + lower * vThick.z };

			pts.push_back( pUpper );
			pts.push_back( pLower );
		}
	}

	return b3CreateHull( pts.data(), (int)pts.size(), 32 );
}

class GliderFlight : public Sample
{
public:
	explicit GliderFlight( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -35.0f, 15.0f, 60.0f, { 0.0f, 12.0f, 0.0f } );
		}

		AddGroundBox( 100.0f );

		m_launchSpeed = 22.0f;
		m_launchPitchDeg = 5.0f;
		m_launchAltitude = 18.0f;

		SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
	}

	void SpawnGlider( b3Pos position )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = position;
		bodyDef.angularDamping = 0.2f;

		float pitchRad = m_launchPitchDeg * B3_PI / 180.0f;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );

		// Initial forward velocity (toward -Z)
		float vz = -m_launchSpeed * cosf( pitchRad );
		float vy = m_launchSpeed * sinf( pitchRad );
		bodyDef.linearVelocity = { 0.0f, vy, vz };

		b3BodyId gliderId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 20.0f;
		shapeDef.enableLift = true;

		// 1. Fuselage: streamlined capsule along Z axis (nose at -Z, tail at +Z)
		b3Capsule fuselage = { { 0.0f, 0.0f, -2.4f }, { 0.0f, 0.15f, 2.4f }, 0.22f };
		b3CreateCapsuleShape( gliderId, &shapeDef, &fuselage );

		// 2. Main Wings: wide cambered airfoil wings spanning across X (from -3.6m to +3.6m)
		// located near aerodynamic center (z = -0.3m, y = 0.22m)
		b3HullData* wingHull = CreateGliderWingHull( 7.2f, 1.3f, 0.12f, 0.035f, 3.5f, { 0.0f, 0.22f, -0.3f } );
		if ( wingHull != nullptr )
		{
			b3ShapeDef wingShape = shapeDef;
			wingShape.density = 10.0f;
			wingShape.airfoil = b3MakeCamberedAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, 7.2f * 1.3f, 8.0f );
			wingShape.airfoil.centerOfPressure = { 0.0f, 0.22f, -0.3f };
			b3CreateHullShape( gliderId, &wingShape, wingHull );
			b3DestroyHull( wingHull );
		}

		// 3. Horizontal Stabilizer / Elevator: rear wing across X at the tail (z = +2.2m)
		b3HullData* hStabHull = CreateBoxHullOffset( { 1.1f, 0.02f, 0.35f }, { 0.0f, 0.25f, 2.1f } );
		if ( hStabHull != nullptr )
		{
			b3ShapeDef tailDef = shapeDef;
			tailDef.density = 8.0f;
			tailDef.airfoil = b3MakeSymmetricAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, 2.2f * 0.35f, 6.0f );
			tailDef.airfoil.centerOfPressure = { 0.0f, 0.25f, 2.1f };
			b3CreateHullShape( gliderId, &tailDef, hStabHull );
			b3DestroyHull( hStabHull );
		}

		// 4. Vertical Stabilizer / Rudder Fin: swept vertical fin at the tail (z = +2.0m, y pointing up)
		b3HullData* vFinHull = CreateSweptVerticalFinHull( 0.7f, 0.4f, 0.85f, 0.25f, 0.04f, { 0.0f, 0.22f, 2.0f } );
		if ( vFinHull != nullptr )
		{
			b3ShapeDef finDef = shapeDef;
			finDef.density = 8.0f;
			finDef.airfoil = b3MakeSymmetricAirfoil( { 0.0f, 0.0f, -1.0f }, { 1.0f, 0.0f, 0.0f }, 0.85f * 0.4f, 2.5f );
			finDef.airfoil.centerOfPressure = { 0.0f, 0.5f, 2.0f };
			b3CreateHullShape( gliderId, &finDef, vFinHull );
			b3DestroyHull( vFinHull );
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift Demo: Glider Flight" );
		DrawTextLine( "Press Space to launch a new glider." );
		DrawTextLine( "Aerodynamic lift & drag forces are active and driven by wing geometry." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
		}
	}

	bool DrawControls() override
	{
		bool changed = false;
		if ( ImGui::Button( "Launch Glider" ) )
		{
			SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
			changed = true;
		}
		ImGui::SliderFloat( "Launch Speed (m/s)", &m_launchSpeed, 5.0f, 50.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Pitch (deg)", &m_launchPitchDeg, -15.0f, 30.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Altitude (m)", &m_launchAltitude, 5.0f, 50.0f, "%.1f" );
		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new GliderFlight( context );
	}

private:
	float m_launchSpeed;
	float m_launchPitchDeg;
	float m_launchAltitude;
};

static int sampleGliderFlight = RegisterSample( "Aerodynamics", "Glider Flight", GliderFlight::Create );

class WindTunnelAirfoil : public Sample
{
public:
	explicit WindTunnelAirfoil( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -18.0f, 10.0f, 22.0f, { 0.0f, 4.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		m_windSpeed = 25.0f;
		m_angleOfAttackDeg = 8.0f;

		for ( int i = 0; i < 3; ++i )
		{
			m_jointIds[i] = b3_nullJointId;
			m_wingIds[i] = b3_nullBodyId;
		}

		// Mount 3 test airfoils on motorized pivot stations:
		// 1. Cambered Clark-Y style airfoil
		// 2. Symmetric NACA 0012 airfoil
		// 3. Flat plate
		m_labels[0] = "Cambered Airfoil";
		m_labels[1] = "Symmetric 0012";
		m_labels[2] = "Flat Plate";

		SpawnAirfoilStation( 0, { -5.0f, 4.0f, 0.0f }, 0.12f, 0.05f );
		SpawnAirfoilStation( 1, { 0.0f, 4.0f, 0.0f }, 0.12f, 0.00f );
		SpawnAirfoilStation( 2, { 5.0f, 4.0f, 0.0f }, 0.02f, 0.00f );
	}

	void SpawnAirfoilStation( int index, b3Pos pos, float thickness, float camber )
	{
		// Static mount post
		b3BodyDef postDef = b3DefaultBodyDef();
		postDef.position = { pos.x, pos.y * 0.5f, pos.z };
		b3BodyId postId = b3CreateBody( m_worldId, &postDef );
		b3Capsule postCap = { { 0.0f, -pos.y * 0.5f, 0.0f }, { 0.0f, pos.y * 0.5f, 0.0f }, 0.08f };
		b3ShapeDef postShape = b3DefaultShapeDef();
		b3CreateCapsuleShape( postId, &postShape, &postCap );

		// Dynamic airfoil wing section (span along Z, chord along X)
		b3BodyDef wingDef = b3DefaultBodyDef();
		wingDef.type = b3_dynamicBody;
		wingDef.position = pos;
		wingDef.angularDamping = 0.5f;
		b3BodyId wingId = b3CreateBody( m_worldId, &wingDef );
		m_wingIds[index] = wingId;

		// Generate airfoil with chord along X and span along Z
		enum
		{
			kPts = 12
		};
		float chord = 1.6f;
		float span = 2.5f;
		std::vector<b3Vec3> pts;
		pts.reserve( kPts * 4 );

		for ( int i = 0; i < kPts; ++i )
		{
			float t = (float)i / (float)( kPts - 1 );
			float xFrac = 0.5f * ( 1.0f - cosf( t * B3_PI ) );
			float xLocal = ( xFrac - 0.5f ) * chord;
			float yt = 5.0f * thickness * chord *
					   ( 0.2969f * sqrtf( xFrac ) - 0.1260f * xFrac - 0.3516f * xFrac * xFrac +
						 0.2843f * xFrac * xFrac * xFrac - 0.1015f * xFrac * xFrac * xFrac * xFrac );
			float yc = 4.0f * camber * chord * xFrac * ( 1.0f - xFrac );

			pts.push_back( { xLocal, yc + yt, -0.5f * span } );
			pts.push_back( { xLocal, yc - yt, -0.5f * span } );
			pts.push_back( { xLocal, yc + yt, 0.5f * span } );
			pts.push_back( { xLocal, yc - yt, 0.5f * span } );
		}

		b3HullData* hull = b3CreateHull( pts.data(), (int)pts.size(), 32 );
		if ( hull != nullptr )
		{
			b3ShapeDef wingShape = b3DefaultShapeDef();
			wingShape.density = 2.0f;
			wingShape.enableLift = true;

			// Assign appropriate airfoil polar definition
			if ( camber > 0.01f )
			{
				wingShape.airfoil = b3MakeCamberedAirfoil( { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, chord * span, 6.0f );
			}
			else if ( thickness > 0.05f )
			{
				wingShape.airfoil = b3MakeSymmetricAirfoil( { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, chord * span, 6.0f );
			}
			else
			{
				wingShape.airfoil = b3MakeFlatPlateAirfoil( { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, chord * span, 5.0f );
			}

			b3CreateHullShape( wingId, &wingShape, hull );
			b3DestroyHull( hull );
		}

		// Pivot revolute joint along Z axis with stiff servo spring to maintain commanded AoA
		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = postId;
		jointDef.base.bodyIdB = wingId;
		jointDef.base.localFrameA.p = { 0.0f, pos.y * 0.5f, 0.0f };
		jointDef.base.localFrameB.p = b3Vec3_zero;
		jointDef.base.localFrameA.q = b3Quat_identity;
		jointDef.base.localFrameB.q = b3Quat_identity;
		jointDef.enableSpring = true;
		jointDef.hertz = 15.0f;
		jointDef.dampingRatio = 1.0f;
		jointDef.targetAngle = -m_angleOfAttackDeg * ( B3_PI / 180.0f );
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.5f * B3_PI;
		jointDef.upperAngle = 0.5f * B3_PI;
		m_jointIds[index] = b3CreateRevoluteJoint( m_worldId, &jointDef );
	}

	void Step() override
	{
		b3World_SetWind( m_worldId, { m_windSpeed, 0.0f, 0.0f } );

		float targetAngleRad = -m_angleOfAttackDeg * ( B3_PI / 180.0f );
		for ( int i = 0; i < 3; ++i )
		{
			if ( B3_IS_NON_NULL( m_jointIds[i] ) )
			{
				b3RevoluteJoint_SetTargetAngle( m_jointIds[i], targetAngleRad );
			}
		}

		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Wind Tunnel Test Section" );
		DrawTextLine( "Simulated Wind Speed: %.1f m/s | Commanded AoA: %.1f deg", m_windSpeed, m_angleOfAttackDeg );

		for ( int i = 0; i < 3; ++i )
		{
			if ( B3_IS_NON_NULL( m_jointIds[i] ) )
			{
				b3Vec3 rForce = b3Joint_GetConstraintForce( m_jointIds[i] );
				float dragForce = fabsf( rForce.x );
				float liftForce = fabsf( rForce.y );
				float ldRatio = dragForce > 0.01f ? ( liftForce / dragForce ) : 0.0f;
				DrawTextLine( "[%s] Lift: %.1f N | Drag: %.1f N | L/D: %.2f", m_labels[i], liftForce, dragForce, ldRatio );
			}
		}
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Wind Speed (m/s)", &m_windSpeed, 0.0f, 60.0f, "%.1f" );
		ImGui::SliderFloat( "Target AoA (deg)", &m_angleOfAttackDeg, -30.0f, 30.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new WindTunnelAirfoil( context );
	}

private:
	float m_windSpeed;
	float m_angleOfAttackDeg;
	b3JointId m_jointIds[3];
	b3BodyId m_wingIds[3];
	const char* m_labels[3];
};

static int sampleWindTunnel = RegisterSample( "Aerodynamics", "Wind Tunnel Airfoil", WindTunnelAirfoil::Create );

class AssignableAirfoils : public Sample
{
public:
	explicit AssignableAirfoils( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -25.0f, 12.0f, 45.0f, { 0.0f, 8.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		m_launchSpeed = 20.0f;
		m_launchPitchDeg = 6.0f;
		m_launchHeight = 16.0f;

		SpawnAirfoilBoxes();
	}

	void SpawnAirfoilBoxes()
	{
		float pitchRad = m_launchPitchDeg * B3_PI / 180.0f;
		float vz = -m_launchSpeed * cosf( pitchRad );
		float vy = m_launchSpeed * sinf( pitchRad );

		// 1. Box with Cambered Airfoil Profile
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -6.0f, m_launchHeight, 20.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );
			bodyDef.linearVelocity = { 0.0f, vy, vz };
			bodyDef.angularDamping = 0.3f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 1.5f, 0.15f, 0.6f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 15.0f;
			shapeDef.enableLift = true;
			shapeDef.airfoil = b3MakeCamberedAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, 3.0f * 1.2f, 7.0f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		// 2. Box with Symmetric Airfoil Profile
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -2.0f, m_launchHeight, 20.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );
			bodyDef.linearVelocity = { 0.0f, vy, vz };
			bodyDef.angularDamping = 0.3f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 1.5f, 0.15f, 0.6f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 15.0f;
			shapeDef.enableLift = true;
			shapeDef.airfoil = b3MakeSymmetricAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, 3.0f * 1.2f, 7.0f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		// 3. Box with Flat Plate Airfoil Profile
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 2.0f, m_launchHeight, 20.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );
			bodyDef.linearVelocity = { 0.0f, vy, vz };
			bodyDef.angularDamping = 0.3f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 1.5f, 0.15f, 0.6f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 15.0f;
			shapeDef.enableLift = true;
			shapeDef.airfoil = b3MakeFlatPlateAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, 3.0f * 1.2f, 5.0f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		// 4. Box with Proxy NACA Wing Hull Aerodynamics
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 6.0f, m_launchHeight, 20.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );
			bodyDef.linearVelocity = { 0.0f, vy, vz };
			bodyDef.angularDamping = 0.3f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 1.5f, 0.15f, 0.6f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 15.0f;
			shapeDef.enableLift = true;

			b3HullData* nacaProxy = CreateGliderWingHull( 3.0f, 1.2f, 0.12f, 0.04f, 0.0f, { 0.0f, 0.0f, 0.0f } );
			if ( nacaProxy != nullptr )
			{
				shapeDef.airfoil = b3MakeProxyHullAirfoil( nacaProxy );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
				b3DestroyHull( nacaProxy );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Assignable Airfoils: Custom Aerodynamics on Simple Collision Boxes" );
		DrawTextLine( "Boxes visually identical, but carry assigned Cambered, Symmetric, Flat Plate, and Proxy NACA polars." );
		DrawTextLine( "Press Space to relaunch all test boxes." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnAirfoilBoxes();
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Relaunch Boxes" ) )
		{
			SpawnAirfoilBoxes();
			return true;
		}
		ImGui::SliderFloat( "Launch Speed (m/s)", &m_launchSpeed, 5.0f, 40.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Pitch (deg)", &m_launchPitchDeg, -15.0f, 25.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Height (m)", &m_launchHeight, 5.0f, 40.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new AssignableAirfoils( context );
	}

private:
	float m_launchSpeed;
	float m_launchPitchDeg;
	float m_launchHeight;
};

static int sampleAssignableAirfoils = RegisterSample( "Aerodynamics", "Assignable Airfoils", AssignableAirfoils::Create );

class SpinningRotor : public Sample
{
public:
	explicit SpinningRotor( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -25.0f, 15.0f, 35.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		m_bladeCount = 3;
		m_initialRPM = 240.0f;
		m_dropHeight = 25.0f;

		SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
	}

	void SpawnRotorAssembly( b3Pos pos )
	{
		// 1. Central Payload / Mast body
		b3BodyDef mastDef = b3DefaultBodyDef();
		mastDef.type = b3_dynamicBody;
		mastDef.position = pos;
		mastDef.linearDamping = 0.05f;
		mastDef.angularDamping = 0.3f;
		b3BodyId mastId = b3CreateBody( m_worldId, &mastDef );

		b3Capsule payloadCap = { { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.2f, 0.0f }, 0.25f };
		b3ShapeDef mastShape = b3DefaultShapeDef();
		mastShape.density = 20.0f;
		b3CreateCapsuleShape( mastId, &mastShape, &payloadCap );

		// 2. Rotating Rotor Hub (revolute joint along vertical Y axis)
		b3BodyDef hubDef = b3DefaultBodyDef();
		hubDef.type = b3_dynamicBody;
		hubDef.allowFastRotation = true;
		hubDef.position = { pos.x, pos.y + 0.45f, pos.z };

		// Initial angular spin around Y
		float omegaY = m_initialRPM * ( 2.0f * B3_PI / 60.0f );
		hubDef.angularVelocity = { 0.0f, omegaY, 0.0f };
		b3BodyId hubId = b3CreateBody( m_worldId, &hubDef );

		// Hub central cylinder
		b3ShapeDef hubShape = b3DefaultShapeDef();
		hubShape.density = 2.0f;
		b3HullData* hubCylinder = b3CreateCylinder( 0.08f, 0.25f, 0.0f, 16 );
		if ( hubCylinder != nullptr )
		{
			b3CreateHullShape( hubId, &hubShape, hubCylinder );
			b3DestroyHull( hubCylinder );
		}

		// Attach radial pitched rotor blades directly as compound shapes to the Hub
		float innerR = 0.25f;
		float outerR = 2.0f;
		float bladeChord = 0.25f;
		float pitchAngleDeg = 6.0f; // positive collective pitch

		b3ShapeDef bladeShape = b3DefaultShapeDef();
		bladeShape.density = 0.5f;
		bladeShape.enableLift = true;

		for ( int i = 0; i < m_bladeCount; ++i )
		{
			float azimuthRad = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
			b3HullData* bladeHull =
				CreateRadialRotorBladeHull( innerR, outerR, bladeChord, pitchAngleDeg, azimuthRad );
			if ( bladeHull != nullptr )
			{
				b3CreateHullShape( hubId, &bladeShape, bladeHull );
				b3DestroyHull( bladeHull );
			}
		}

		// Connect hub to mast via low-friction Revolute Joint around vertical Y axis
		b3RevoluteJointDef mastJoint = b3DefaultRevoluteJointDef();
		mastJoint.base.bodyIdA = mastId;
		mastJoint.base.bodyIdB = hubId;
		mastJoint.base.localFrameA.p = { 0.0f, 0.45f, 0.0f };
		mastJoint.base.localFrameB.p = b3Vec3_zero;
		// Revolute joint default axis is local Z, rotate to Y
		mastJoint.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
		mastJoint.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
		b3CreateRevoluteJoint( m_worldId, &mastJoint );
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Spinning Rotor / Autogyro" );
		DrawTextLine( "Blades are pitched and attached radially to the rotating hub." );
		DrawTextLine( "Press Space to spawn another rotor assembly." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Spawn Rotor" ) )
		{
			SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
			return true;
		}
		ImGui::SliderFloat( "Initial RPM", &m_initialRPM, 0.0f, 800.0f, "%.0f" );
		ImGui::SliderFloat( "Drop Height (m)", &m_dropHeight, 5.0f, 50.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpinningRotor( context );
	}

private:
	int m_bladeCount;
	float m_initialRPM;
	float m_dropHeight;
};

static int sampleSpinningRotor = RegisterSample( "Aerodynamics", "Spinning Rotor", SpinningRotor::Create );

class FallingAirfoils : public Sample
{
public:
	explicit FallingAirfoils( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -20.0f, 15.0f, 40.0f, { 0.0f, 8.0f, 0.0f } );
		}

		AddGroundBox( 30.0f );

		SpawnShapes();
	}

	void SpawnShapes()
	{
		for ( int i = -3; i <= 3; ++i )
		{
			float x = (float)i * 3.5f;

			// 1. Cambered Airfoil (span across X, chord along Z)
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x, 16.0f, -4.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (float)i * 0.15f );
				bodyDef.angularDamping = 0.2f;
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3HullData* hull =
					CreateGliderWingHull( 2.5f, 1.4f, 0.12f, 0.04f, 0.0f, { 0.0f, 0.0f, 0.0f } );
				if ( hull != nullptr )
				{
					b3ShapeDef shapeDef = b3DefaultShapeDef();
					shapeDef.density = 15.0f;
					shapeDef.enableLift = true;
					b3CreateHullShape( bodyId, &shapeDef, hull );
					b3DestroyHull( hull );
				}
			}

			// 2. Flat Plate
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x, 14.0f, 0.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, (float)i * 0.2f );
				bodyDef.angularDamping = 0.2f;
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3BoxHull plate = b3MakeBoxHull( 1.0f, 0.015f, 0.8f );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				shapeDef.density = 15.0f;
				shapeDef.enableLift = true;
				b3CreateHullShape( bodyId, &shapeDef, &plate.base );
			}

			// 3. Disc / Frisbee
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.allowFastRotation = true;
				bodyDef.position = { x, 18.0f, 4.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.1f * (float)i );
				bodyDef.angularVelocity = { 0.0f, 40.0f, 0.0f };
				bodyDef.angularDamping = 0.1f;
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3HullData* discHull = b3CreateCylinder( 0.04f, 0.7f, 0.0f, 16 );
				if ( discHull != nullptr )
				{
					b3ShapeDef shapeDef = b3DefaultShapeDef();
					shapeDef.density = 15.0f;
					shapeDef.enableLift = true;
					b3CreateHullShape( bodyId, &shapeDef, discHull );
					b3DestroyHull( discHull );
				}
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Falling Airfoils, Plates & Discs" );
		DrawTextLine( "Aerodynamic drag & lift forces are actively simulated based on shape geometry." );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FallingAirfoils( context );
	}
};

static int sampleFallingAirfoils = RegisterSample( "Aerodynamics", "Falling Airfoils", FallingAirfoils::Create );

class FinStabilizedDarts : public Sample
{
public:
	explicit FinStabilizedDarts( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -40.0f, 12.0f, 50.0f, { 0.0f, 6.0f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		m_launchSpeed = 40.0f;

		for ( int i = 0; i < 5; ++i )
		{
			LaunchDart( { -25.0f, 4.0f + (float)i * 2.0f, (float)( i - 2 ) * 3.0f } );
		}
	}

	void LaunchDart( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.linearVelocity = { m_launchSpeed, 2.0f, 0.0f }; // high speed along +X
		bodyDef.angularDamping = 0.5f;

		b3BodyId dartId = b3CreateBody( m_worldId, &bodyDef );

		// 1. Slender body shaft (capsule along X from tail at x=-0.9 to front at x=+0.5)
		b3Capsule shaft = { { -0.9f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.035f };
		b3ShapeDef shaftShape = b3DefaultShapeDef();
		shaftShape.density = 80.0f;
		shaftShape.enableLift = true;
		b3CreateCapsuleShape( dartId, &shaftShape, &shaft );

		// 2. Heavy nose tip at the front (+X) for forward CG stability
		b3Sphere nose = { { 0.55f, 0.0f, 0.0f }, 0.055f };
		b3ShapeDef noseShape = b3DefaultShapeDef();
		noseShape.density = 800.0f;
		noseShape.enableLift = true;
		b3CreateSphereShape( dartId, &noseShape, &nose );

		// 3. Four stabilizing swept delta fins positioned at the TAIL end (x in [-0.9f, -0.6f])
		b3ShapeDef finShape = b3DefaultShapeDef();
		finShape.density = 20.0f;
		finShape.enableLift = true;

		for ( int orientation = 0; orientation < 4; ++orientation )
		{
			b3HullData* finHull = CreateDartSingleFinHull( -0.6f, -0.9f, 0.035f, 0.16f, 0.008f, orientation );
			if ( finHull != nullptr )
			{
				b3CreateHullShape( dartId, &finShape, finHull );
				b3DestroyHull( finHull );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Fin-Stabilized Darts" );
		DrawTextLine( "Swept stabilizing fins are at the rear tail end." );
		DrawTextLine( "Press Space to launch more darts." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			LaunchDart( { -25.0f, 6.0f, (float)( ( m_stepCount % 5 ) - 2 ) * 2.5f } );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Launch Darts" ) )
		{
			for ( int i = 0; i < 5; ++i )
			{
				LaunchDart( { -25.0f, 4.0f + (float)i * 2.0f, (float)( i - 2 ) * 3.0f } );
			}
			return true;
		}
		ImGui::SliderFloat( "Dart Speed (m/s)", &m_launchSpeed, 10.0f, 80.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new FinStabilizedDarts( context );
	}

private:
	float m_launchSpeed;
};

static int sampleDarts = RegisterSample( "Aerodynamics", "Fin Stabilized Darts", FinStabilizedDarts::Create );

class VoxelAerodynamics : public Sample
{
public:
	explicit VoxelAerodynamics( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -30.0f, 15.0f, 45.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 60.0f );

		m_launchSpeed = 20.0f;
		m_windSpeed = 0.0f;

		// 1. Spawn a Voxel Delta Wing Glider
		SpawnVoxelGlider( { 0.0f, 18.0f, 25.0f } );

		// 2. Spawn a group of falling thin voxel plates at varied angles
		for ( int i = -2; i <= 2; ++i )
		{
			SpawnVoxelPlate( { (float)i * 4.0f, 14.0f + (float)abs( i ) * 2.0f, -5.0f }, (float)i * 0.25f );
		}
	}

	void SpawnVoxelGlider( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.angularDamping = 0.2f;
		// Initial speed toward -Z with slight climb
		bodyDef.linearVelocity = { 0.0f, 2.0f, -m_launchSpeed };

		b3BodyId gliderId = b3CreateBody( m_worldId, &bodyDef );

		// Build a swept delta wing with central fuselage from voxel lattice
		std::vector<b3Vec3i> cells;
		float voxelSize = 0.25f;

		// Fuselage along Z (from z = -6 to +6)
		for ( int z = -6; z <= 6; ++z )
		{
			cells.push_back( { 0, 0, z } );
			cells.push_back( { 0, 1, z } );
		}
		// Nose weight
		cells.push_back( { 0, 0, -7 } );

		// Swept wings extending out in X and swept back toward +Z
		for ( int x = 1; x <= 14; ++x )
		{
			int zLeading = -4 + x / 2;
			int chord = 8 - x / 3;
			for ( int z = zLeading; z < zLeading + chord; ++z )
			{
				cells.push_back( { x, 0, z } );
				cells.push_back( { -x, 0, z } );
			}
		}

		// Vertical stabilizer fins at wing tips
		for ( int y = 1; y <= 4; ++y )
		{
			cells.push_back( { 14, y, 5 } );
			cells.push_back( { -14, y, 5 } );
		}

		b3VoxelData* voxelData = b3CreateVoxelData( cells.data(), (int)cells.size(), voxelSize );
		if ( voxelData != nullptr )
		{
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 15.0f;
			shapeDef.enableLift = true;
			b3CreateVoxelShape( gliderId, &shapeDef, voxelData );
		}
	}

	void SpawnVoxelPlate( b3Pos pos, float angle )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, angle );
		bodyDef.angularDamping = 0.2f;

		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		// 8x1x8 thin voxel plate
		std::vector<b3Vec3i> cells;
		float voxelSize = 0.3f;
		for ( int x = -4; x <= 4; ++x )
		{
			for ( int z = -4; z <= 4; ++z )
			{
				cells.push_back( { x, 0, z } );
			}
		}

		b3VoxelData* voxelData = b3CreateVoxelData( cells.data(), (int)cells.size(), voxelSize );
		if ( voxelData != nullptr )
		{
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 10.0f;
			shapeDef.enableLift = true;
			b3CreateVoxelShape( bodyId, &shapeDef, voxelData );
		}
	}

	void Step() override
	{
		b3World_SetWind( m_worldId, { 0.0f, 0.0f, -m_windSpeed } );

		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Voxel Aerodynamics & Gliders" );
		DrawTextLine( "Voxel lattice geometry calculates surface boundary exposures in real time." );
		DrawTextLine( "Press Space to launch another voxel glider." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnVoxelGlider( { 0.0f, 18.0f, 25.0f } );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Launch Voxel Glider" ) )
		{
			SpawnVoxelGlider( { 0.0f, 18.0f, 25.0f } );
			return true;
		}
		if ( ImGui::Button( "Drop Voxel Plates" ) )
		{
			for ( int i = -2; i <= 2; ++i )
			{
				SpawnVoxelPlate( { (float)i * 4.0f, 16.0f, -5.0f }, (float)i * 0.25f );
			}
			return true;
		}
		ImGui::SliderFloat( "Glider Speed (m/s)", &m_launchSpeed, 5.0f, 50.0f, "%.1f" );
		ImGui::SliderFloat( "Headwind (m/s)", &m_windSpeed, -20.0f, 30.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new VoxelAerodynamics( context );
	}

private:
	float m_launchSpeed;
	float m_windSpeed;
};

static int sampleVoxelAerodynamics = RegisterSample( "Aerodynamics", "Voxel Aerodynamics", VoxelAerodynamics::Create );

class StormSimulation : public Sample
{
public:
	explicit StormSimulation( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -35.0f, 20.0f, 45.0f, { 0.0f, 5.0f, 0.0f } );
		}

		m_groundId = AddGroundBox( 80.0f );

		m_windSpeed = 26.0f;
		m_windAngleDeg = 45.0f;
		m_windPitchDeg = 3.0f;
		m_gustiness = 0.40f;
		m_time = 0.0f;
		m_debrisCount = 0;
		m_presetIndex = 3; // Severe Storm

		ResetEnvironment();
	}

	void ResetEnvironment()
	{
		// Spawn a grove of stylized, stable spring-jointed trees
		struct TreePlacement
		{
			b3Pos pos;
			float scale;
		};

		const TreePlacement trees[] = {
			{ { -14.0f, 0.0f, -10.0f }, 1.1f },
			{ { -10.0f, 0.0f, -2.0f }, 1.3f },
			{ { -16.0f, 0.0f, 8.0f }, 0.9f },
			{ { -6.0f, 0.0f, 12.0f }, 1.2f },
			{ { -4.0f, 0.0f, -16.0f }, 1.0f },
			{ { 5.0f, 0.0f, -14.0f }, 1.15f },
			{ { 12.0f, 0.0f, -8.0f }, 0.95f },
			{ { 16.0f, 0.0f, 3.0f }, 1.25f },
			{ { 10.0f, 0.0f, 10.0f }, 1.05f },
			{ { 4.0f, 0.0f, 18.0f }, 0.85f },
			{ { -2.0f, 0.0f, 6.0f }, 1.0f },
			{ { -20.0f, 0.0f, -4.0f }, 1.15f },
		};

		for ( const auto& t : trees )
		{
			SpawnTree( t.pos, t.scale );
		}

		// Shed structure with loose roof sheet metal panels
		SpawnShedStructure( { 0.0f, 0.0f, -2.0f } );

		// Initial ground clutter debris
		for ( int i = 0; i < 6; ++i )
		{
			float rx = ( (float)( ( i * 37 ) % 26 ) - 13.0f );
			float rz = ( (float)( ( i * 53 ) % 26 ) - 13.0f );
			if ( i % 3 == 0 )
			{
				SpawnSheetMetal( { rx, 1.0f, rz }, (float)i * 0.4f );
			}
			else if ( i % 3 == 1 )
			{
				SpawnCrate( { rx, 0.8f, rz }, 0.6f );
			}
			else
			{
				SpawnBarrel( { rx, 0.7f, rz } );
			}
		}
	}

	// Helper to spawn a clean, stylized tree with angular spring weld joints
	void SpawnTree( b3Pos rootPos, float scale )
	{
		float trunkHalfH = 1.8f * scale;
		float trunkRadius = 0.22f * scale;

		// 1. Lower Trunk
		b3BodyDef trunkDef = b3DefaultBodyDef();
		trunkDef.type = b3_dynamicBody;
		trunkDef.position = { rootPos.x, trunkHalfH, rootPos.z };
		trunkDef.linearDamping = 0.3f;
		trunkDef.angularDamping = 0.8f;
		b3BodyId trunkId = b3CreateBody( m_worldId, &trunkDef );

		b3Capsule trunkCap = { { 0.0f, -trunkHalfH + trunkRadius, 0.0f },
							   { 0.0f, trunkHalfH - trunkRadius, 0.0f },
							   trunkRadius };
		b3ShapeDef trunkShape = b3DefaultShapeDef();
		trunkShape.density = 25.0f;
		b3CreateCapsuleShape( trunkId, &trunkShape, &trunkCap );

		// Root spring weld joint to ground (rigid position, very stiff angular spring)
		b3WeldJointDef rootJoint = b3DefaultWeldJointDef();
		rootJoint.base.bodyIdA = m_groundId;
		rootJoint.base.bodyIdB = trunkId;
		rootJoint.base.localFrameA.p = { rootPos.x, 0.0f, rootPos.z };
		rootJoint.base.localFrameB.p = { 0.0f, -trunkHalfH, 0.0f };
		rootJoint.base.localFrameA.q = b3Quat_identity;
		rootJoint.base.localFrameB.q = b3Quat_identity;
		rootJoint.linearHertz = 0.0f;
		rootJoint.linearDampingRatio = 0.0f;
		rootJoint.angularHertz = 60.0f / sqrtf( scale );
		rootJoint.angularDampingRatio = 0.85f;
		rootJoint.base.drawScale = 0.3f;
		b3CreateWeldJoint( m_worldId, &rootJoint );

		// 2. Upper Canopy Segment
		float canopyHalfH = 1.5f * scale;
		b3BodyDef canopyDef = b3DefaultBodyDef();
		canopyDef.type = b3_dynamicBody;
		canopyDef.position = { rootPos.x, 2.0f * trunkHalfH + canopyHalfH, rootPos.z };
		canopyDef.linearDamping = 0.4f;
		canopyDef.angularDamping = 0.8f;
		b3BodyId canopyId = b3CreateBody( m_worldId, &canopyDef );

		// Stylized tiered foliage boxes
		b3BoxHull foliage1 = b3MakeOffsetBoxHull( 1.5f * scale, 0.5f * scale, 1.5f * scale, { 0.0f, -0.4f * scale, 0.0f } );
		b3BoxHull foliage2 = b3MakeOffsetBoxHull( 1.1f * scale, 0.5f * scale, 1.1f * scale, { 0.0f, 0.4f * scale, 0.0f } );
		b3BoxHull foliage3 = b3MakeOffsetBoxHull( 0.7f * scale, 0.4f * scale, 0.7f * scale, { 0.0f, 1.1f * scale, 0.0f } );

		b3ShapeDef foliageShape = b3DefaultShapeDef();
		foliageShape.density = 0.3f; // Lightweight leaf foliage
		foliageShape.enableLift = true;
		float foliageArea = 0.6f * scale * scale;
		foliageShape.airfoil = b3MakeFlatPlateAirfoil( { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, foliageArea, 1.2f );

		b3CreateHullShape( canopyId, &foliageShape, &foliage1.base );
		b3CreateHullShape( canopyId, &foliageShape, &foliage2.base );
		b3CreateHullShape( canopyId, &foliageShape, &foliage3.base );

		// Middle trunk-to-canopy spring weld joint
		b3WeldJointDef midJoint = b3DefaultWeldJointDef();
		midJoint.base.bodyIdA = trunkId;
		midJoint.base.bodyIdB = canopyId;
		midJoint.base.localFrameA.p = { 0.0f, trunkHalfH, 0.0f };
		midJoint.base.localFrameB.p = { 0.0f, -canopyHalfH, 0.0f };
		midJoint.base.localFrameA.q = b3Quat_identity;
		midJoint.base.localFrameB.q = b3Quat_identity;
		midJoint.linearHertz = 0.0f;
		midJoint.linearDampingRatio = 0.0f;
		midJoint.angularHertz = 45.0f / sqrtf( scale );
		midJoint.angularDampingRatio = 0.85f;
		midJoint.base.drawScale = 0.3f;
		b3CreateWeldJoint( m_worldId, &midJoint );
	}

	// Helper to spawn a wooden shed structure with loose roof sheet metal panels
	void SpawnShedStructure( b3Pos center )
	{
		float width = 4.0f;
		float depth = 5.0f;
		float postHeight = 3.0f;

		// 4 static corner posts
		for ( float sx : { -0.5f, 0.5f } )
		{
			for ( float sz : { -0.5f, 0.5f } )
			{
				b3BodyDef postDef = b3DefaultBodyDef();
				postDef.position = { center.x + sx * width, 0.5f * postHeight, center.z + sz * depth };
				b3BodyId postId = b3CreateBody( m_worldId, &postDef );
				b3BoxHull box = b3MakeBoxHull( 0.15f, 0.5f * postHeight, 0.15f );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateHullShape( postId, &shapeDef, &box.base );
			}
		}

		// Cross beams
		b3BodyDef beamDef = b3DefaultBodyDef();
		beamDef.position = { center.x, postHeight, center.z };
		b3BodyId beamId = b3CreateBody( m_worldId, &beamDef );
		b3BoxHull beamL = b3MakeOffsetBoxHull( 0.1f, 0.1f, 0.5f * depth, { -0.5f * width, 0.0f, 0.0f } );
		b3BoxHull beamR = b3MakeOffsetBoxHull( 0.1f, 0.1f, 0.5f * depth, { 0.5f * width, 0.0f, 0.0f } );
		b3ShapeDef beamShape = b3DefaultShapeDef();
		b3CreateHullShape( beamId, &beamShape, &beamL.base );
		b3CreateHullShape( beamId, &beamShape, &beamR.base );

		// Loose roof sheet metal plates resting on top
		for ( int i = 0; i < 4; ++i )
		{
			float zOffset = ( (float)i - 1.5f ) * 1.2f;
			b3BodyDef sheetDef = b3DefaultBodyDef();
			sheetDef.type = b3_dynamicBody;
			sheetDef.position = { center.x, postHeight + 0.15f, center.z + zOffset };
			sheetDef.linearDamping = 0.05f;
			sheetDef.angularDamping = 0.1f;
			b3BodyId sheetId = b3CreateBody( m_worldId, &sheetDef );

			b3BoxHull sheetHull = b3MakeBoxHull( 0.55f * width, 0.02f, 0.55f );
			b3ShapeDef sheetShape = b3DefaultShapeDef();
			sheetShape.density = 2.0f; // Lightweight corrugated sheet
			sheetShape.enableLift = true;
			sheetShape.baseMaterial.friction = 0.6f;
			sheetShape.airfoil = b3MakeFlatPlateAirfoil( { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, width * 1.1f, 3.5f );
			b3CreateHullShape( sheetId, &sheetShape, &sheetHull.base );
		}
	}

	// Debris Spawners
	void SpawnSheetMetal( b3Pos pos, float rotDeg )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, rotDeg * B3_DEG_TO_RAD );
		bodyDef.linearDamping = 0.02f;
		bodyDef.angularDamping = 0.05f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull hull = b3MakeBoxHull( 0.9f, 0.015f, 0.6f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.8f;
		shapeDef.enableLift = true;
		shapeDef.airfoil = b3MakeFlatPlateAirfoil( { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 1.8f * 1.2f, 1.5f );
		b3CreateHullShape( bodyId, &shapeDef, &hull.base );
		m_debrisCount++;
	}

	void SpawnCrate( b3Pos pos, float halfSize )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.linearDamping = 0.05f;
		bodyDef.angularDamping = 0.08f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull hull = b3MakeCubeHull( halfSize );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 7.0f;
		shapeDef.enableLift = true;
		b3CreateHullShape( bodyId, &shapeDef, &hull.base );
		m_debrisCount++;
	}

	void SpawnBarrel( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 0.0f, 1.0f }, 90.0f * B3_DEG_TO_RAD );
		bodyDef.linearDamping = 0.03f;
		bodyDef.angularDamping = 0.03f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3Capsule cap = { { -0.4f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.0f }, 0.35f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 10.0f;
		shapeDef.enableLift = true;
		shapeDef.baseMaterial.rollingResistance = 0.01f;
		b3CreateCapsuleShape( bodyId, &shapeDef, &cap );
		m_debrisCount++;
	}

	void SpawnStormGlider( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		float span = 3.0f;
		float chord = 1.2f;
		b3BoxHull wing = b3MakeBoxHull( 0.5f * span, 0.025f, 0.5f * chord );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.5f;
		shapeDef.enableLift = true;
		shapeDef.airfoil = b3MakeCamberedAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, span * chord, 5.0f );
		b3CreateHullShape( bodyId, &shapeDef, &wing.base );
		m_debrisCount++;
	}

	void SpawnDebrisSwarm( int count )
	{
		float radH = m_windAngleDeg * B3_DEG_TO_RAD;
		b3Vec3 upwindDir = { -sinf( radH ), 0.0f, -cosf( radH ) };

		for ( int i = 0; i < count; ++i )
		{
			float dist = 22.0f + (float)( ( i * 7 ) % 15 );
			float spread = ( (float)( i % 7 ) - 3.0f ) * 3.5f;
			b3Vec3 perp = { -upwindDir.z, 0.0f, upwindDir.x };

			b3Pos spawnPos = { upwindDir.x * dist + perp.x * spread, 5.0f + (float)( ( i * 11 ) % 10 ),
							   upwindDir.z * dist + perp.z * spread };

			int type = i % 3;
			if ( type == 0 )
			{
				SpawnSheetMetal( spawnPos, (float)( i * 45 ) );
			}
			else if ( type == 1 )
			{
				SpawnCrate( spawnPos, 0.45f + 0.1f * (float)( i % 3 ) );
			}
			else
			{
				SpawnBarrel( spawnPos );
			}
		}
	}

	const char* GetBeaufortDescription( float speedMs ) const
	{
		if ( speedMs < 0.5f )
			return "Calm";
		if ( speedMs < 3.3f )
			return "Light Breeze";
		if ( speedMs < 7.9f )
			return "Moderate Breeze";
		if ( speedMs < 13.8f )
			return "Strong Breeze";
		if ( speedMs < 17.1f )
			return "Near Gale";
		if ( speedMs < 20.7f )
			return "Gale";
		if ( speedMs < 24.4f )
			return "Strong Gale";
		if ( speedMs < 28.4f )
			return "Storm (Whole Gale)";
		if ( speedMs < 32.6f )
			return "Violent Storm";
		return "Hurricane Force";
	}

	void Step() override
	{
		m_time += 1.0f / 60.0f;

		// Turbulent multi-frequency storm gusting
		float noise = 0.50f * sinf( 1.7f * m_time ) +
					  0.35f * sinf( 3.1f * m_time ) +
					  0.15f * sinf( 0.7f * m_time );
		float gustFactor = b3MaxFloat( 0.0f, 1.0f + m_gustiness * noise );
		float currentSpeed = m_windSpeed * gustFactor;

		// Compute 3D wind velocity vector
		float radH = m_windAngleDeg * B3_DEG_TO_RAD;
		float radP = m_windPitchDeg * B3_DEG_TO_RAD;
		float cosP = cosf( radP );
		float sinP = sinf( radP );

		b3Vec3 windDir = { cosP * sinf( radH ), sinP, cosP * cosf( radH ) };
		b3Vec3 windVector = b3MulSV( currentSpeed, windDir );

		b3World_SetWind( m_worldId, windVector );

		Sample::Step();

		// Wind direction arrow in the sky
		b3Pos arrowOrigin = { 0.0f, 15.0f, 0.0f };
		b3Vec3 arrowDelta = b3MulSV( b3MinFloat( currentSpeed * 0.25f, 12.0f ), windDir );
		b3Pos arrowTip = b3Add( arrowOrigin, arrowDelta );
		Vec4 arrowColor = { 0.3f, 0.8f, 1.0f, 0.9f };
		DrawArrow( arrowOrigin, arrowTip, arrowColor );

		// On-screen storm telemetry
		float speedKmh = currentSpeed * 3.6f;

		DrawTextLine( "Shape-Based Lift: Storm & Debris Simulation" );
		DrawTextLine( "Wind: %.1f m/s (%.1f km/h) | Heading: %.0f deg | Updraft: %.1f deg", currentSpeed, speedKmh,
					  m_windAngleDeg, m_windPitchDeg );
		DrawTextLine( "Status: %s | Active Debris: %d", GetBeaufortDescription( currentSpeed ), m_debrisCount );
		DrawTextLine( "Press [Space] to spawn debris swarm | [G] launch glider" );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS )
		{
			if ( key == KEY_SPACE )
			{
				SpawnDebrisSwarm( 12 );
			}
			else if ( key == SAPP_KEYCODE_G )
			{
				float radH = m_windAngleDeg * B3_DEG_TO_RAD;
				b3Pos launchPos = { -sinf( radH ) * 18.0f, 12.0f, -cosf( radH ) * 18.0f };
				SpawnStormGlider( launchPos );
			}
		}
	}

	bool DrawControls() override
	{
		bool changed = false;

		const char* presets[] = { "Custom", "Calm (0 m/s)", "Gentle Breeze (8 m/s)", "Moderate Gale (18 m/s)",
								  "Severe Storm (28 m/s)", "Hurricane (48 m/s)" };
		if ( ImGui::Combo( "Preset", &m_presetIndex, presets, IM_ARRAYSIZE( presets ) ) )
		{
			switch ( m_presetIndex )
			{
				case 1: // Calm
					m_windSpeed = 0.0f;
					m_gustiness = 0.0f;
					m_windPitchDeg = 0.0f;
					break;
				case 2: // Breeze
					m_windSpeed = 8.0f;
					m_gustiness = 0.20f;
					m_windPitchDeg = 1.0f;
					break;
				case 3: // Gale
					m_windSpeed = 18.0f;
					m_gustiness = 0.35f;
					m_windPitchDeg = 2.0f;
					break;
				case 4: // Storm
					m_windSpeed = 28.0f;
					m_gustiness = 0.50f;
					m_windPitchDeg = 4.0f;
					break;
				case 5: // Hurricane
					m_windSpeed = 48.0f;
					m_gustiness = 0.65f;
					m_windPitchDeg = 6.0f;
					break;
			}
		}

		if ( ImGui::SliderFloat( "Wind Speed (m/s)", &m_windSpeed, 0.0f, 65.0f, "%.1f" ) )
		{
			m_presetIndex = 0; // Custom
		}
		if ( ImGui::SliderFloat( "Wind Heading (deg)", &m_windAngleDeg, 0.0f, 360.0f, "%.0f" ) )
		{
			m_presetIndex = 0;
		}
		if ( ImGui::SliderFloat( "Updraft Pitch (deg)", &m_windPitchDeg, -10.0f, 25.0f, "%.1f" ) )
		{
			m_presetIndex = 0;
		}
		ImGui::SliderFloat( "Gustiness", &m_gustiness, 0.0f, 1.0f, "%.2f" );

		ImGui::Separator();
		if ( ImGui::Button( "Spawn Debris Swarm" ) )
		{
			SpawnDebrisSwarm( 12 );
			changed = true;
		}
		if ( ImGui::Button( "Launch Storm Glider" ) )
		{
			float radH = m_windAngleDeg * B3_DEG_TO_RAD;
			b3Pos launchPos = { -sinf( radH ) * 18.0f, 12.0f, -cosf( radH ) * 18.0f };
			SpawnStormGlider( launchPos );
			changed = true;
		}

		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new StormSimulation( context );
	}

private:
	b3BodyId m_groundId;
	float m_windSpeed;
	float m_windAngleDeg;
	float m_windPitchDeg;
	float m_gustiness;
	float m_time;
	int m_debrisCount;
	int m_presetIndex;
};

static int sampleStorm = RegisterSample( "Aerodynamics", "Storm and Debris", StormSimulation::Create );

class FlexibleTree : public Sample
{
public:
	explicit FlexibleTree( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -18.0f, 8.0f, 22.0f, { 0.0f, 4.0f, 0.0f } );
		}

		m_groundId = AddGroundBox( 40.0f );

		m_windSpeed = 8.0f;
		m_windAngleDeg = 45.0f;
		m_windPitchDeg = 2.0f;
		m_gustiness = 0.35f;
		m_flexScale = 1.0f;
		m_time = 0.0f;
		m_gustPulseTimer = 0.0f;
		m_leafCount = 0;
		m_branchCount = 0;

		BuildTree();
	}

	// Helper to rotate local +Y vector into a given 3D unit direction
	static b3Quat MakeQuatFromYAxis( b3Vec3 dir )
	{
		b3Vec3 up = { 0.0f, 1.0f, 0.0f };
		b3Vec3 axis = b3Cross( up, dir );
		float dot = b3Dot( up, dir );
		if ( b3LengthSquared( axis ) < 1e-6f )
		{
			if ( dot < 0.0f )
			{
				return b3MakeQuatFromAxisAngle( { 1.0f, 0.0f, 0.0f }, B3_PI );
			}
			return b3Quat_identity;
		}
		axis = b3Normalize( axis );
		float angle = acosf( b3ClampFloat( dot, -1.0f, 1.0f ) );
		return b3MakeQuatFromAxisAngle( axis, angle );
	}

	void AddWeldJoint( b3BodyId bodyA, b3BodyId bodyB, b3Pos anchorA, b3Pos anchorB, float baseHertz,
					   float dampingRatio = 0.85f )
	{
		b3WeldJointDef jointDef = b3DefaultWeldJointDef();
		jointDef.base.bodyIdA = bodyA;
		jointDef.base.bodyIdB = bodyB;
		jointDef.base.localFrameA.p = anchorA;
		jointDef.base.localFrameB.p = anchorB;

		b3Quat qA = b3Body_GetRotation( bodyA );
		b3Quat qB = b3Body_GetRotation( bodyB );
		// Rest orientation matches initial relative rotation
		jointDef.base.localFrameA.q = b3InvMulQuat( qA, qB );
		jointDef.base.localFrameB.q = b3Quat_identity;

		jointDef.linearHertz = 0.0f;
		jointDef.linearDampingRatio = 0.0f;
		jointDef.angularHertz = baseHertz / sqrtf( m_flexScale );
		jointDef.angularDampingRatio = dampingRatio;
		jointDef.base.drawScale = 0.15f;

		b3JointId jointId = b3CreateWeldJoint( m_worldId, &jointDef );
		m_joints.push_back( { jointId, baseHertz, dampingRatio } );
	}

	void BuildTree()
	{
		m_joints.clear();
		m_leafCount = 0;
		m_branchCount = 0;

		struct TrunkDef
		{
			float y0;
			float y1;
			float radius;
			float hertz;
		};

		const TrunkDef trunkSegments[4] = {
			{ 0.0f, 1.8f, 0.32f, 70.0f },
			{ 1.8f, 3.4f, 0.28f, 55.0f },
			{ 3.4f, 4.8f, 0.24f, 45.0f },
			{ 4.8f, 6.0f, 0.20f, 40.0f },
		};

		b3BodyId trunkBodies[4];
		b3BodyId prevBody = m_groundId;

		for ( int i = 0; i < 4; ++i )
		{
			float len = trunkSegments[i].y1 - trunkSegments[i].y0;
			float r = trunkSegments[i].radius;
			float yCenter = 0.5f * ( trunkSegments[i].y0 + trunkSegments[i].y1 );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, yCenter, 0.0f };
			bodyDef.linearDamping = 0.3f;
			bodyDef.angularDamping = 0.8f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			trunkBodies[i] = bodyId;

			b3Capsule cap = { { 0.0f, -0.5f * len + r, 0.0f }, { 0.0f, 0.5f * len - r, 0.0f }, r };
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 25.0f - 2.0f * (float)i;
			b3CreateCapsuleShape( bodyId, &shapeDef, &cap );

			if ( i == 0 )
			{
				AddWeldJoint( prevBody, bodyId, { 0.0f, 0.0f, 0.0f }, { 0.0f, -0.5f * len, 0.0f },
							  trunkSegments[i].hertz, 0.85f );
			}
			else
			{
				float prevLen = trunkSegments[i - 1].y1 - trunkSegments[i - 1].y0;
				AddWeldJoint( prevBody, bodyId, { 0.0f, 0.5f * prevLen, 0.0f }, { 0.0f, -0.5f * len, 0.0f },
							  trunkSegments[i].hertz, 0.85f );
			}

			prevBody = bodyId;
			m_branchCount++;
		}

		struct BranchDef
		{
			int trunkIndex;
			float yAttachTrunk; // local Y on trunk body
			float azimuthDeg;
			float pitchDeg;
			float length;
			float radius;
			float hertz;
		};

		const BranchDef branches[] = {
			// Lower branches on Trunk 1
			{ 1, -0.3f, 0.0f, 25.0f, 1.8f, 0.16f, 35.0f },
			{ 1, 0.1f, 120.0f, 28.0f, 1.7f, 0.15f, 35.0f },
			{ 1, 0.4f, 240.0f, 26.0f, 1.8f, 0.16f, 35.0f },
			// Mid branches on Trunk 2
			{ 2, -0.3f, 60.0f, 32.0f, 1.6f, 0.14f, 32.0f },
			{ 2, 0.0f, 180.0f, 30.0f, 1.6f, 0.14f, 32.0f },
			{ 2, 0.3f, 300.0f, 34.0f, 1.5f, 0.13f, 32.0f },
			// Upper crown branches on Trunk 3
			{ 3, -0.2f, 30.0f, 42.0f, 1.4f, 0.12f, 30.0f },
			{ 3, 0.1f, 150.0f, 40.0f, 1.3f, 0.11f, 30.0f },
			{ 3, 0.3f, 270.0f, 44.0f, 1.4f, 0.12f, 30.0f },
		};

		for ( const auto& b : branches )
		{
			b3BodyId parentTrunk = trunkBodies[b.trunkIndex];
			b3Pos trunkPos = b3Body_GetPosition( parentTrunk );

			float radAz = b.azimuthDeg * B3_DEG_TO_RAD;
			float radPitch = b.pitchDeg * B3_DEG_TO_RAD;
			float cosP = cosf( radPitch );
			float sinP = sinf( radPitch );

			b3Vec3 branchDir = { cosP * sinf( radAz ), sinP, cosP * cosf( radAz ) };
			branchDir = b3Normalize( branchDir );

			b3Pos attachWorld = { trunkPos.x + branchDir.x * 0.25f, trunkPos.y + b.yAttachTrunk,
								  trunkPos.z + branchDir.z * 0.25f };
			b3Pos branchCenter = { attachWorld.x + 0.5f * b.length * branchDir.x,
								   attachWorld.y + 0.5f * b.length * branchDir.y,
								   attachWorld.z + 0.5f * b.length * branchDir.z };

			// Spawn Primary Branch Body
			b3BodyDef bDef = b3DefaultBodyDef();
			bDef.type = b3_dynamicBody;
			bDef.position = branchCenter;
			bDef.rotation = MakeQuatFromYAxis( branchDir );
			bDef.linearDamping = 0.3f;
			bDef.angularDamping = 0.8f;
			b3BodyId branchId = b3CreateBody( m_worldId, &bDef );

			b3Capsule bCap = { { 0.0f, -0.5f * b.length + b.radius, 0.0f },
							   { 0.0f, 0.5f * b.length - b.radius, 0.0f },
							   b.radius };
			b3ShapeDef bShape = b3DefaultShapeDef();
			bShape.density = 8.0f;
			b3CreateCapsuleShape( branchId, &bShape, &bCap );

			AddWeldJoint( parentTrunk, branchId, { 0.0f, b.yAttachTrunk, 0.0f }, { 0.0f, -0.5f * b.length, 0.0f },
						  b.hertz, 0.85f );
			m_branchCount++;

			// Spawn 2 Sub-Branches (Twigs) branching left/right from the primary branch tip
			b3Pos branchTipWorld = { attachWorld.x + b.length * branchDir.x, attachWorld.y + b.length * branchDir.y,
									 attachWorld.z + b.length * branchDir.z };

			for ( float forkSign : { -1.0f, 1.0f } )
			{
				float twigAz = ( b.azimuthDeg + forkSign * 32.0f ) * B3_DEG_TO_RAD;
				float twigPitch = ( b.pitchDeg + 12.0f ) * B3_DEG_TO_RAD;
				b3Vec3 twigDir = { cosf( twigPitch ) * sinf( twigAz ), sinf( twigPitch ),
								   cosf( twigPitch ) * cosf( twigAz ) };
				twigDir = b3Normalize( twigDir );

				float twigLen = 1.1f;
				float twigRadius = 0.08f;
				b3Pos twigCenter = { branchTipWorld.x + 0.5f * twigLen * twigDir.x,
									 branchTipWorld.y + 0.5f * twigLen * twigDir.y,
									 branchTipWorld.z + 0.5f * twigLen * twigDir.z };

				b3BodyDef twigDef = b3DefaultBodyDef();
				twigDef.type = b3_dynamicBody;
				twigDef.position = twigCenter;
				twigDef.rotation = MakeQuatFromYAxis( twigDir );
				twigDef.linearDamping = 0.35f;
				twigDef.angularDamping = 0.8f;
				b3BodyId twigId = b3CreateBody( m_worldId, &twigDef );

				b3Capsule twigCap = { { 0.0f, -0.5f * twigLen + twigRadius, 0.0f },
									  { 0.0f, 0.5f * twigLen - twigRadius, 0.0f },
									  twigRadius };
				b3ShapeDef twigShape = b3DefaultShapeDef();
				twigShape.density = 4.0f;
				b3CreateCapsuleShape( twigId, &twigShape, &twigCap );

				AddWeldJoint( branchId, twigId, { 0.0f, 0.5f * b.length, 0.0f }, { 0.0f, -0.5f * twigLen, 0.0f },
							  b.hertz + 3.0f, 0.85f );
				m_branchCount++;

				b3Pos twigTipWorld = { branchTipWorld.x + twigLen * twigDir.x,
									   branchTipWorld.y + twigLen * twigDir.y,
									   branchTipWorld.z + twigLen * twigDir.z };

				b3BodyDef leafDef = b3DefaultBodyDef();
				leafDef.type = b3_dynamicBody;
				leafDef.position = twigTipWorld;
				leafDef.linearDamping = 0.4f;
				leafDef.angularDamping = 0.85f;
				b3BodyId leafId = b3CreateBody( m_worldId, &leafDef );

				// Clean, stylized box foliage tufts
				b3BoxHull leafBox1 = b3MakeBoxHull( 0.55f, 0.25f, 0.55f );
				b3BoxHull leafBox2 = b3MakeOffsetBoxHull( 0.35f, 0.20f, 0.35f, { 0.0f, 0.25f, 0.0f } );

				b3ShapeDef leafShape = b3DefaultShapeDef();
				leafShape.density = 0.3f; // Lightweight leaf clusters
				leafShape.enableLift = true;
				float leafArea = 0.25f; // Realistic aerodynamic area
				leafShape.airfoil = b3MakeFlatPlateAirfoil( { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, leafArea, 1.2f );

				b3CreateHullShape( leafId, &leafShape, &leafBox1.base );
				b3CreateHullShape( leafId, &leafShape, &leafBox2.base );

				AddWeldJoint( twigId, leafId, { 0.0f, 0.5f * twigLen, 0.0f }, { 0.0f, 0.0f, 0.0f }, 22.0f, 0.8f );
				m_leafCount++;
			}
		}

		// Top crown foliage tuft at the top of Trunk 3
		{
			b3BodyId topTrunk = trunkBodies[3];
			b3Pos trunkPos = b3Body_GetPosition( topTrunk );
			b3Pos crownPos = { trunkPos.x, trunkPos.y + 0.6f, trunkPos.z };

			b3BodyDef crownDef = b3DefaultBodyDef();
			crownDef.type = b3_dynamicBody;
			crownDef.position = crownPos;
			crownDef.linearDamping = 0.4f;
			crownDef.angularDamping = 0.85f;
			b3BodyId crownId = b3CreateBody( m_worldId, &crownDef );

			b3BoxHull crownBox = b3MakeBoxHull( 0.7f, 0.4f, 0.7f );
			b3ShapeDef crownShape = b3DefaultShapeDef();
			crownShape.density = 0.3f;
			crownShape.enableLift = true;
			crownShape.airfoil = b3MakeFlatPlateAirfoil( { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, 0.4f, 1.2f );
			b3CreateHullShape( crownId, &crownShape, &crownBox.base );

			AddWeldJoint( topTrunk, crownId, { 0.0f, 0.6f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 24.0f, 0.8f );
			m_leafCount++;
		}
	}

	void UpdateJointStiffness()
	{
		for ( const auto& j : m_joints )
		{
			float hertz = j.baseHertz / sqrtf( m_flexScale );
			b3WeldJoint_SetAngularHertz( j.id, hertz );
			b3WeldJoint_SetAngularDampingRatio( j.id, j.dampingRatio );
			b3Joint_WakeBodies( j.id );
		}
	}

	void SpawnFruitDrop()
	{
		for ( int i = 0; i < 8; ++i )
		{
			float rx = ( (float)( ( i * 37 ) % 16 ) - 8.0f ) * 0.25f;
			float rz = ( (float)( ( i * 53 ) % 16 ) - 8.0f ) * 0.25f;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { rx, 7.5f + (float)i * 0.4f, rz };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.15f };
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 25.0f;
			shapeDef.baseMaterial.restitution = 0.4f;
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
	}

	void Step() override
	{
		m_time += 1.0f / 60.0f;

		// Turbulent storm gusting
		float noise = 0.55f * sinf( 1.8f * m_time ) + 0.30f * sinf( 3.3f * m_time ) + 0.15f * sinf( 0.8f * m_time );
		float gustFactor = b3MaxFloat( 0.0f, 1.0f + m_gustiness * noise );

		// Check sudden impulse gust blast
		if ( m_gustPulseTimer > 0.0f )
		{
			m_gustPulseTimer -= 1.0f / 60.0f;
			gustFactor += 1.8f;
		}

		float currentSpeed = m_windSpeed * gustFactor;

		// Compute 3D wind velocity vector
		float radH = m_windAngleDeg * B3_DEG_TO_RAD;
		float radP = m_windPitchDeg * B3_DEG_TO_RAD;
		float cosP = cosf( radP );
		float sinP = sinf( radP );

		b3Vec3 windDir = { cosP * sinf( radH ), sinP, cosP * cosf( radH ) };
		b3Vec3 windVector = b3MulSV( currentSpeed, windDir );

		b3World_SetWind( m_worldId, windVector );

		Sample::Step();

		// Wind direction arrow in the sky
		b3Pos arrowOrigin = { 0.0f, 10.0f, 0.0f };
		b3Vec3 arrowDelta = b3MulSV( b3MinFloat( currentSpeed * 0.3f, 8.0f ), windDir );
		b3Pos arrowTip = b3Add( arrowOrigin, arrowDelta );
		Vec4 arrowColor = { 0.3f, 0.8f, 1.0f, 0.9f };
		DrawArrow( arrowOrigin, arrowTip, arrowColor );

		// Telemetry
		float speedKmh = currentSpeed * 3.6f;
		DrawTextLine( "Shape-Based Lift: High-Detail Articulated Flexible Tree" );
		DrawTextLine( "Wind: %.1f m/s (%.1f km/h) | Heading: %.0f deg | Gust: x%.2f", currentSpeed, speedKmh,
					  m_windAngleDeg, gustFactor );
		DrawTextLine( "Tree Structure: %d Branches/Twigs | %d Foliage Clusters", m_branchCount, m_leafCount );
		DrawTextLine( "Flexibility Scale: %.2fx | Press [Space] for Storm Blast | [B] Drop Fruit", m_flexScale );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS )
		{
			if ( key == KEY_SPACE )
			{
				m_gustPulseTimer = 1.2f;
			}
			else if ( key == KEY_B )
			{
				SpawnFruitDrop();
			}
		}
	}

	bool DrawControls() override
	{
		bool changed = false;

		ImGui::SliderFloat( "Wind Speed (m/s)", &m_windSpeed, 0.0f, 45.0f, "%.1f" );
		ImGui::SliderFloat( "Wind Heading (deg)", &m_windAngleDeg, 0.0f, 360.0f, "%.0f" );
		ImGui::SliderFloat( "Updraft Pitch (deg)", &m_windPitchDeg, -10.0f, 20.0f, "%.1f" );
		ImGui::SliderFloat( "Gustiness", &m_gustiness, 0.0f, 1.0f, "%.2f" );

		ImGui::Separator();
		if ( ImGui::SliderFloat( "Tree Flexibility", &m_flexScale, 0.2f, 2.5f, "%.2fx" ) )
		{
			UpdateJointStiffness();
		}

		ImGui::Separator();
		if ( ImGui::Button( "Trigger Storm Blast (Space)" ) )
		{
			m_gustPulseTimer = 1.2f;
		}
		if ( ImGui::Button( "Drop Fruit / Apples (B)" ) )
		{
			SpawnFruitDrop();
			changed = true;
		}
		if ( ImGui::Button( "Rebuild Tree" ) )
		{
			// Reset tree
			CreateWorld( nullptr );
			m_groundId = AddGroundBox( 40.0f );
			BuildTree();
			changed = true;
		}

		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new FlexibleTree( context );
	}

private:
	struct JointInfo
	{
		b3JointId id;
		float baseHertz;
		float dampingRatio;
	};

	b3BodyId m_groundId;
	std::vector<JointInfo> m_joints;
	float m_windSpeed;
	float m_windAngleDeg;
	float m_windPitchDeg;
	float m_gustiness;
	float m_flexScale;
	float m_time;
	float m_gustPulseTimer;
	int m_leafCount;
	int m_branchCount;
};

static int sampleFlexibleTree = RegisterSample( "Aerodynamics", "Flexible Tree", FlexibleTree::Create );

class TornadoSimulation : public Sample
{
public:
	explicit TornadoSimulation( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -38.0f, 22.0f, 48.0f, { 0.0f, 8.0f, 0.0f } );
		}

		m_groundId = AddGroundBox( 90.0f );

		m_tornadoPos = { 0.0f, 0.0f };
		m_swirlSpeed = 50.0f;
		m_suctionSpeed = 16.0f;
		m_updraftSpeed = 28.0f;
		m_coreRadius = 3.5f;
		m_maxRadius = 35.0f;
		m_funnelFlaring = 0.22f;
		m_autoWander = true;
		m_wanderSpeed = 3.5f;
		m_time = 0.0f;
		m_debrisCount = 0;
		m_gliderCount = 0;

		ResetEnvironment();
	}

	void ResetEnvironment()
	{
		m_dynamicBodies.clear();
		m_debrisCount = 0;
		m_gliderCount = 0;

		// 1. Destructible Wooden Shed with loose sheet metal roof panels
		SpawnShedStructure( { -8.0f, 0.0f, -4.0f } );
		SpawnShedStructure( { 10.0f, 0.0f, 8.0f } );

		// 2. Spring-Jointed Articulated Trees
		const b3Pos treePositions[] = {
			{ -18.0f, 0.0f, -12.0f },
			{ -14.0f, 0.0f, 15.0f },
			{ 16.0f, 0.0f, -15.0f },
			{ 22.0f, 0.0f, 10.0f },
			{ -5.0f, 0.0f, -22.0f },
			{ 6.0f, 0.0f, 22.0f },
		};
		for ( const auto& p : treePositions )
		{
			SpawnTree( p, 1.1f );
		}

		// 3. Ground Debris & Gliders
		SpawnGroundClutter();
	}

	void SpawnTree( b3Pos rootPos, float scale )
	{
		float trunkHalfH = 1.8f * scale;
		float trunkRadius = 0.22f * scale;

		b3BodyDef trunkDef = b3DefaultBodyDef();
		trunkDef.type = b3_dynamicBody;
		trunkDef.position = { rootPos.x, trunkHalfH, rootPos.z };
		trunkDef.linearDamping = 0.3f;
		trunkDef.angularDamping = 0.8f;
		b3BodyId trunkId = b3CreateBody( m_worldId, &trunkDef );
		m_dynamicBodies.push_back( trunkId );

		b3Capsule trunkCap = { { 0.0f, -trunkHalfH + trunkRadius, 0.0f },
							   { 0.0f, trunkHalfH - trunkRadius, 0.0f },
							   trunkRadius };
		b3ShapeDef trunkShape = b3DefaultShapeDef();
		trunkShape.density = 25.0f;
		b3CreateCapsuleShape( trunkId, &trunkShape, &trunkCap );

		b3WeldJointDef rootJoint = b3DefaultWeldJointDef();
		rootJoint.base.bodyIdA = m_groundId;
		rootJoint.base.bodyIdB = trunkId;
		rootJoint.base.localFrameA.p = { rootPos.x, 0.0f, rootPos.z };
		rootJoint.base.localFrameB.p = { 0.0f, -trunkHalfH, 0.0f };
		rootJoint.linearHertz = 0.0f;
		rootJoint.angularHertz = 60.0f / sqrtf( scale );
		rootJoint.angularDampingRatio = 0.85f;
		rootJoint.base.drawScale = 0.3f;
		b3CreateWeldJoint( m_worldId, &rootJoint );

		// Canopy
		float canopyHalfH = 1.5f * scale;
		b3BodyDef canopyDef = b3DefaultBodyDef();
		canopyDef.type = b3_dynamicBody;
		canopyDef.position = { rootPos.x, 2.0f * trunkHalfH + canopyHalfH, rootPos.z };
		canopyDef.linearDamping = 0.4f;
		canopyDef.angularDamping = 0.8f;
		b3BodyId canopyId = b3CreateBody( m_worldId, &canopyDef );
		m_dynamicBodies.push_back( canopyId );

		b3BoxHull foliage1 = b3MakeOffsetBoxHull( 1.5f * scale, 0.5f * scale, 1.5f * scale, { 0.0f, -0.4f * scale, 0.0f } );
		b3BoxHull foliage2 = b3MakeOffsetBoxHull( 1.1f * scale, 0.5f * scale, 1.1f * scale, { 0.0f, 0.4f * scale, 0.0f } );

		b3ShapeDef foliageShape = b3DefaultShapeDef();
		foliageShape.density = 0.3f;
		foliageShape.enableLift = true;
		float foliageArea = 0.8f * scale * scale;
		foliageShape.airfoil = b3MakeFlatPlateAirfoil( { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, foliageArea, 1.2f );

		b3CreateHullShape( canopyId, &foliageShape, &foliage1.base );
		b3CreateHullShape( canopyId, &foliageShape, &foliage2.base );

		b3WeldJointDef midJoint = b3DefaultWeldJointDef();
		midJoint.base.bodyIdA = trunkId;
		midJoint.base.bodyIdB = canopyId;
		midJoint.base.localFrameA.p = { 0.0f, trunkHalfH, 0.0f };
		midJoint.base.localFrameB.p = { 0.0f, -canopyHalfH, 0.0f };
		midJoint.linearHertz = 0.0f;
		midJoint.angularHertz = 45.0f / sqrtf( scale );
		midJoint.angularDampingRatio = 0.85f;
		midJoint.base.drawScale = 0.3f;
		b3CreateWeldJoint( m_worldId, &midJoint );
	}

	void SpawnShedStructure( b3Pos center )
	{
		float width = 4.0f;
		float depth = 5.0f;
		float postHeight = 3.0f;

		// Static posts
		for ( float sx : { -0.5f, 0.5f } )
		{
			for ( float sz : { -0.5f, 0.5f } )
			{
				b3BodyDef postDef = b3DefaultBodyDef();
				postDef.position = { center.x + sx * width, 0.5f * postHeight, center.z + sz * depth };
				b3BodyId postId = b3CreateBody( m_worldId, &postDef );
				b3BoxHull box = b3MakeBoxHull( 0.15f, 0.5f * postHeight, 0.15f );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateHullShape( postId, &shapeDef, &box.base );
			}
		}

		// Cross beams
		b3BodyDef beamDef = b3DefaultBodyDef();
		beamDef.position = { center.x, postHeight, center.z };
		b3BodyId beamId = b3CreateBody( m_worldId, &beamDef );
		b3BoxHull beamL = b3MakeOffsetBoxHull( 0.1f, 0.1f, 0.5f * depth, { -0.5f * width, 0.0f, 0.0f } );
		b3BoxHull beamR = b3MakeOffsetBoxHull( 0.1f, 0.1f, 0.5f * depth, { 0.5f * width, 0.0f, 0.0f } );
		b3ShapeDef beamShape = b3DefaultShapeDef();
		b3CreateHullShape( beamId, &beamShape, &beamL.base );
		b3CreateHullShape( beamId, &beamShape, &beamR.base );

		// Loose roof sheets
		for ( int i = 0; i < 4; ++i )
		{
			float zOffset = ( (float)i - 1.5f ) * 1.2f;
			b3BodyDef sheetDef = b3DefaultBodyDef();
			sheetDef.type = b3_dynamicBody;
			sheetDef.position = { center.x, postHeight + 0.15f, center.z + zOffset };
			sheetDef.linearDamping = 0.05f;
			sheetDef.angularDamping = 0.1f;
			b3BodyId sheetId = b3CreateBody( m_worldId, &sheetDef );
			m_dynamicBodies.push_back( sheetId );

			b3BoxHull sheetHull = b3MakeBoxHull( 0.55f * width, 0.02f, 0.55f );
			b3ShapeDef sheetShape = b3DefaultShapeDef();
			sheetShape.density = 2.0f;
			sheetShape.enableLift = true;
			sheetShape.baseMaterial.friction = 0.6f;
			sheetShape.airfoil = b3MakeFlatPlateAirfoil( { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, width * 1.1f, 3.5f );
			b3CreateHullShape( sheetId, &sheetShape, &sheetHull.base );
			m_debrisCount++;
		}
	}

	void SpawnSheetMetal( b3Pos pos, float rotDeg )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, rotDeg * B3_DEG_TO_RAD );
		bodyDef.linearDamping = 0.02f;
		bodyDef.angularDamping = 0.05f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		m_dynamicBodies.push_back( bodyId );

		b3BoxHull hull = b3MakeBoxHull( 0.9f, 0.015f, 0.6f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.8f;
		shapeDef.enableLift = true;
		shapeDef.airfoil = b3MakeFlatPlateAirfoil( { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 1.8f * 1.2f, 1.5f );
		b3CreateHullShape( bodyId, &shapeDef, &hull.base );
		m_debrisCount++;
	}

	void SpawnCrate( b3Pos pos, float halfSize )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.linearDamping = 0.05f;
		bodyDef.angularDamping = 0.08f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		m_dynamicBodies.push_back( bodyId );

		b3BoxHull hull = b3MakeCubeHull( halfSize );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 7.0f;
		shapeDef.enableLift = true;
		b3CreateHullShape( bodyId, &shapeDef, &hull.base );
		m_debrisCount++;
	}

	void SpawnBarrel( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 0.0f, 1.0f }, 90.0f * B3_DEG_TO_RAD );
		bodyDef.linearDamping = 0.03f;
		bodyDef.angularDamping = 0.03f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		m_dynamicBodies.push_back( bodyId );

		b3Capsule cap = { { -0.4f, 0.0f, 0.0f }, { 0.4f, 0.0f, 0.0f }, 0.35f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 10.0f;
		shapeDef.enableLift = true;
		shapeDef.baseMaterial.rollingResistance = 0.01f;
		b3CreateCapsuleShape( bodyId, &shapeDef, &cap );
		m_debrisCount++;
	}

	void SpawnGlider( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.linearDamping = 0.01f;
		bodyDef.angularDamping = 0.04f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		m_dynamicBodies.push_back( bodyId );

		float span = 3.2f;
		float chord = 1.1f;
		b3BoxHull wing = b3MakeBoxHull( 0.5f * span, 0.02f, 0.5f * chord );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.2f;
		shapeDef.enableLift = true;
		shapeDef.airfoil = b3MakeCamberedAirfoil( { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f }, span * chord, 5.5f );
		b3CreateHullShape( bodyId, &shapeDef, &wing.base );
		m_gliderCount++;
	}

	void SpawnGroundClutter()
	{
		for ( int i = 0; i < 18; ++i )
		{
			float rx = ( (float)( ( i * 47 ) % 40 ) - 20.0f );
			float rz = ( (float)( ( i * 71 ) % 40 ) - 20.0f );
			if ( i % 3 == 0 )
			{
				SpawnSheetMetal( { rx, 0.5f, rz }, (float)( i * 35 ) );
			}
			else if ( i % 3 == 1 )
			{
				SpawnCrate( { rx, 0.5f, rz }, 0.45f + 0.1f * (float)( i % 3 ) );
			}
			else
			{
				SpawnBarrel( { rx, 0.5f, rz } );
			}
		}

		for ( int i = 0; i < 4; ++i )
		{
			float angle = (float)i * 90.0f * B3_DEG_TO_RAD;
			b3Pos gPos = { 12.0f * cosf( angle ), 2.0f, 12.0f * sinf( angle ) };
			SpawnGlider( gPos );
		}
	}

	void SpawnDebrisSwarm( int count )
	{
		for ( int i = 0; i < count; ++i )
		{
			float angle = (float)( ( i * 37 ) % 360 ) * B3_DEG_TO_RAD;
			float r = 5.0f + (float)( ( i * 19 ) % 12 );
			b3Pos spawnPos = { m_tornadoPos.x + r * cosf( angle ), 1.5f + (float)( i % 4 ) * 1.5f,
							   m_tornadoPos.y + r * sinf( angle ) };

			int type = i % 4;
			if ( type == 0 )
			{
				SpawnSheetMetal( spawnPos, (float)( i * 45 ) );
			}
			else if ( type == 1 )
			{
				SpawnCrate( spawnPos, 0.45f + 0.15f * (float)( i % 2 ) );
			}
			else if ( type == 2 )
			{
				SpawnBarrel( spawnPos );
			}
			else
			{
				SpawnGlider( spawnPos );
			}
		}
	}

	b3Vec3 ComputeTornadoWind( b3Pos point ) const
	{
		float dx = point.x - m_tornadoPos.x;
		float dz = point.z - m_tornadoPos.y;
		float r = sqrtf( dx * dx + dz * dz );
		float y = b3MaxFloat( 0.0f, point.y );

		if ( r > m_maxRadius || y > 45.0f )
		{
			return b3Vec3_zero;
		}

		float flaringRadius = m_coreRadius + m_funnelFlaring * y;
		float invR = ( r > 1e-4f ) ? 1.0f / r : 0.0f;

		// Unit tangential vector (counter-clockwise around Y axis)
		b3Vec3 tangDir = { -dz * invR, 0.0f, dx * invR };
		// Unit inward radial vector
		b3Vec3 inDir = { -dx * invR, 0.0f, -dz * invR };

		// 1. Tangential Swirl (Modified Rankine Vortex)
		float vTang = 0.0f;
		if ( r <= flaringRadius )
		{
			vTang = m_swirlSpeed * ( r / flaringRadius );
		}
		else
		{
			float ratio = flaringRadius / r;
			float outerFade = b3MaxFloat( 0.0f, 1.0f - ( r - flaringRadius ) / ( m_maxRadius - flaringRadius ) );
			vTang = m_swirlSpeed * ratio * outerFade;
		}

		// 2. Radial Inflow Suction (strongest near surface and outer edges)
		float altitudeFactor = b3MaxFloat( 0.0f, 1.0f - y / 35.0f );
		float rSuction = b3MinFloat( 1.0f, r / flaringRadius );
		float vSuction = m_suctionSpeed * rSuction * altitudeFactor;

		// 3. Vertical Updraft (strongest inside the core)
		float coreProximity = 1.0f / ( 1.0f + ( r / flaringRadius ) * ( r / flaringRadius ) );
		float verticalRamp = b3MinFloat( 1.0f, y / 3.0f + 0.3f );
		float vUp = m_updraftSpeed * coreProximity * verticalRamp;

		b3Vec3 wind = b3Add( b3MulSV( vTang, tangDir ), b3MulSV( vSuction, inDir ) );
		wind.y += vUp;
		return wind;
	}

	void Step() override
	{
		m_time += 1.0f / 60.0f;

		// Move tornado along a wandering path
		if ( m_autoWander )
		{
			float w = 0.22f * m_wanderSpeed;
			m_tornadoPos.x = 16.0f * sinf( w * m_time ) + 6.0f * sinf( 2.3f * w * m_time );
			m_tornadoPos.y = 12.0f * cosf( 0.8f * w * m_time ) + 5.0f * cosf( 1.9f * w * m_time );
		}

		// Ensure global wind is zero so all wind forces originate from spatial b3Shape_ApplyWind
		b3World_SetWind( m_worldId, b3Vec3_zero );

		// Apply spatial non-global wind to every dynamic shape
		b3ShapeId shapes[16];
		for ( auto bodyId : m_dynamicBodies )
		{
			if ( b3Body_IsValid( bodyId ) == false )
			{
				continue;
			}

			int shapeCount = b3Body_GetShapes( bodyId, shapes, 16 );
			for ( int s = 0; s < shapeCount; ++s )
			{
				b3AABB aabb = b3Shape_GetAABB( shapes[s] );
				b3Pos centroid = b3MulSV( 0.5f, b3Add( aabb.lowerBound, aabb.upperBound ) );

				b3Vec3 localWind = ComputeTornadoWind( centroid );
				if ( b3LengthSquared( localWind ) > 0.25f )
				{
					b3Shape_ApplyWind( shapes[s], localWind, 1.0f, 1.0f, 150.0f, /*wake=*/true );
				}
			}
		}

		Sample::Step();

		// Draw Tornado Funnel Visualizer
		const float ringHeights[] = { 0.2f, 1.5f, 4.0f, 8.0f, 13.0f, 19.0f, 26.0f, 34.0f };
		int numRings = sizeof( ringHeights ) / sizeof( ringHeights[0] );
		const int segments = 24;

		for ( int hIdx = 0; hIdx < numRings; ++hIdx )
		{
			float y = ringHeights[hIdx];
			float r = m_coreRadius + m_funnelFlaring * y;
			float alpha = 0.7f - 0.45f * ( (float)hIdx / (float)numRings );
			Vec4 ringColor = { 0.4f, 0.85f, 1.0f, alpha };

			float spinOffset = m_time * ( 3.0f + 2.0f * ( 1.0f - y / 34.0f ) );

			b3Pos prevPt = { m_tornadoPos.x + r * cosf( spinOffset ), y, m_tornadoPos.y + r * sinf( spinOffset ) };

			for ( int i = 1; i <= segments; ++i )
			{
				float theta = spinOffset + (float)i * ( 2.0f * B3_PI / (float)segments );
				b3Pos currPt = { m_tornadoPos.x + r * cosf( theta ), y, m_tornadoPos.y + r * sinf( theta ) };

				DrawLine( prevPt, currPt, ringColor );

				// Streamer arrows around the funnel perimeter
				if ( i % 6 == 0 )
				{
					b3Vec3 tangent = { -sinf( theta ), 0.2f, cosf( theta ) };
					b3Pos arrowTip = b3Add( currPt, b3MulSV( 1.8f, tangent ) );
					Vec4 arrowCol = { 0.6f, 0.95f, 1.0f, alpha * 0.9f };
					DrawArrow( currPt, arrowTip, arrowCol );
				}

				prevPt = currPt;
			}
		}

		// Vertical Core Spiral Streams
		for ( int sIdx = 0; sIdx < 4; ++sIdx )
		{
			float streamOffset = (float)sIdx * ( 0.5f * B3_PI );
			b3Pos prevStream = { m_tornadoPos.x + m_coreRadius * cosf( streamOffset + m_time * 4.0f ), 0.2f,
								 m_tornadoPos.y + m_coreRadius * sinf( streamOffset + m_time * 4.0f ) };

			for ( int step = 1; step <= 16; ++step )
			{
				float frac = (float)step / 16.0f;
				float y = frac * 32.0f;
				float r = m_coreRadius + m_funnelFlaring * y;
				float theta = streamOffset + m_time * 4.0f + frac * 4.0f * B3_PI;

				b3Pos currStream = { m_tornadoPos.x + r * cosf( theta ), y, m_tornadoPos.y + r * sinf( theta ) };
				Vec4 streamCol = { 0.8f, 0.95f, 1.0f, 0.5f * ( 1.0f - frac ) };
				DrawLine( prevStream, currStream, streamCol );
				prevStream = currStream;
			}
		}

		// Telemetry
		float peakSpeedKmh = sqrtf( m_swirlSpeed * m_swirlSpeed + m_updraftSpeed * m_updraftSpeed ) * 3.6f;
		DrawTextLine( "Shape-Based Lift: Spatial Non-Global Wind - Tornado Vortex" );
		DrawTextLine( "Tornado Center: (%.1f, %.1f) | Peak Core Wind: %.1f km/h", m_tornadoPos.x, m_tornadoPos.y,
					  peakSpeedKmh );
		DrawTextLine( "Debris Count: %d | Gliders in Air: %d", m_debrisCount, m_gliderCount );
		DrawTextLine( "Press [Space] Spawn Debris | [G] Launch Gliders | [V] Spawn Vehicles/Barrels" );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS )
		{
			if ( key == KEY_SPACE )
			{
				SpawnDebrisSwarm( 14 );
			}
			else if ( key == SAPP_KEYCODE_G )
			{
				for ( int i = 0; i < 3; ++i )
				{
					float angle = (float)i * 120.0f * B3_DEG_TO_RAD;
					b3Pos launchPos = { m_tornadoPos.x + 8.0f * cosf( angle ), 4.0f + (float)i * 2.0f,
										m_tornadoPos.y + 8.0f * sinf( angle ) };
					SpawnGlider( launchPos );
				}
			}
			else if ( key == SAPP_KEYCODE_V )
			{
				for ( int i = 0; i < 8; ++i )
				{
					float rx = m_tornadoPos.x + ( (float)( ( i * 37 ) % 16 ) - 8.0f );
					float rz = m_tornadoPos.y + ( (float)( ( i * 53 ) % 16 ) - 8.0f );
					if ( i % 2 == 0 )
					{
						SpawnBarrel( { rx, 1.0f + (float)i * 0.5f, rz } );
					}
					else
					{
						SpawnCrate( { rx, 1.0f + (float)i * 0.5f, rz }, 0.8f );
					}
				}
			}
		}
	}

	bool DrawControls() override
	{
		bool changed = false;

		ImGui::SliderFloat( "Swirl Speed (m/s)", &m_swirlSpeed, 0.0f, 90.0f, "%.1f" );
		ImGui::SliderFloat( "Suction Speed (m/s)", &m_suctionSpeed, 0.0f, 40.0f, "%.1f" );
		ImGui::SliderFloat( "Updraft Speed (m/s)", &m_updraftSpeed, 0.0f, 60.0f, "%.1f" );
		ImGui::SliderFloat( "Core Radius (m)", &m_coreRadius, 1.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Funnel Flaring", &m_funnelFlaring, 0.05f, 0.5f, "%.2f" );

		ImGui::Separator();
		ImGui::Checkbox( "Auto-Wander Path", &m_autoWander );
		if ( m_autoWander )
		{
			ImGui::SliderFloat( "Wander Speed", &m_wanderSpeed, 0.5f, 10.0f, "%.1f" );
		}
		else
		{
			ImGui::SliderFloat( "Tornado X", &m_tornadoPos.x, -30.0f, 30.0f, "%.1f" );
			ImGui::SliderFloat( "Tornado Z", &m_tornadoPos.y, -30.0f, 30.0f, "%.1f" );
		}

		ImGui::Separator();
		if ( ImGui::Button( "Spawn Debris Swarm (Space)" ) )
		{
			SpawnDebrisSwarm( 14 );
			changed = true;
		}
		if ( ImGui::Button( "Launch Gliders (G)" ) )
		{
			for ( int i = 0; i < 3; ++i )
			{
				float angle = (float)i * 120.0f * B3_DEG_TO_RAD;
				b3Pos launchPos = { m_tornadoPos.x + 8.0f * cosf( angle ), 4.0f + (float)i * 2.0f,
									m_tornadoPos.y + 8.0f * sinf( angle ) };
				SpawnGlider( launchPos );
			}
			changed = true;
		}
		if ( ImGui::Button( "Reset Scene" ) )
		{
			CreateWorld( nullptr );
			m_groundId = AddGroundBox( 90.0f );
			ResetEnvironment();
			changed = true;
		}

		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new TornadoSimulation( context );
	}

private:
	b3BodyId m_groundId;
	std::vector<b3BodyId> m_dynamicBodies;
	b3Vec2 m_tornadoPos;
	float m_swirlSpeed;
	float m_suctionSpeed;
	float m_updraftSpeed;
	float m_coreRadius;
	float m_maxRadius;
	float m_funnelFlaring;
	bool m_autoWander;
	float m_wanderSpeed;
	float m_time;
	int m_debrisCount;
	int m_gliderCount;
};

static int sampleTornado = RegisterSample( "Aerodynamics", "Tornado Vortex", TornadoSimulation::Create );

class PropellerThrust : public Sample
{
public:
	explicit PropellerThrust( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -16.0f, 8.0f, 20.0f, { 0.0f, 2.5f, 0.0f } );
		}

		m_groundId = AddGroundBox( 60.0f );

		m_mode = 0; // 0 = Coaxial VTOL Drone, 1 = Guided Thrust Stand, 2 = Runway Propeller Cart
		m_throttle = 0.55f;
		m_maxRPM = 1800.0f;
		m_bladePitchDeg = 12.0f;
		m_bladeCount = 3;
		m_propRadius = 1.2f;
		m_bladeChord = 0.18f;
		m_pitchTrim = 0.0f;
		m_rollTrim = 0.0f;
		m_time = 0.0f;

		m_chassisId = b3_nullBodyId;
		m_hub1Id = b3_nullBodyId;
		m_hub2Id = b3_nullBodyId;
		m_motor1 = b3_nullJointId;
		m_motor2 = b3_nullJointId;
		m_guideJoint = b3_nullJointId;

		BuildVehicle();
	}

	float GetCurrentRadianSpeed() const
	{
		float rpm = m_throttle * m_maxRPM;
		return rpm * ( 2.0f * B3_PI / 60.0f );
	}

	void UpdateMotorSpeeds()
	{
		float radSpeed = GetCurrentRadianSpeed();
		if ( B3_IS_NON_NULL( m_motor1 ) )
		{
			float speed = ( m_mode == 2 ) ? -radSpeed : radSpeed;
			b3RevoluteJoint_SetMotorSpeed( m_motor1, speed );
			b3Joint_WakeBodies( m_motor1 );
		}
		if ( B3_IS_NON_NULL( m_motor2 ) )
		{
			b3RevoluteJoint_SetMotorSpeed( m_motor2, -radSpeed );
			b3Joint_WakeBodies( m_motor2 );
		}
	}

	void BuildVehicle()
	{
		CreateWorld( nullptr );
		m_groundId = AddGroundBox( 60.0f );

		m_chassisId = b3_nullBodyId;
		m_hub1Id = b3_nullBodyId;
		m_hub2Id = b3_nullBodyId;
		m_motor1 = b3_nullJointId;
		m_motor2 = b3_nullJointId;
		m_guideJoint = b3_nullJointId;

		if ( m_mode == 0 )
		{
			b3Pos startPos = { 0.0f, 1.2f, 0.0f };

			// 1. Central Drone Chassis
			b3BodyDef chassisDef = b3DefaultBodyDef();
			chassisDef.type = b3_dynamicBody;
			chassisDef.position = startPos;
			chassisDef.linearDamping = 0.15f;
			chassisDef.angularDamping = 0.8f;
			m_chassisId = b3CreateBody( m_worldId, &chassisDef );

			// Fuselage pod
			b3Capsule podCap = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.35f };
			b3ShapeDef podShape = b3DefaultShapeDef();
			podShape.density = 20.0f;
			podShape.filter.categoryBits = 0x0001;
			b3CreateCapsuleShape( m_chassisId, &podShape, &podCap );

			// Landing skids
			b3ShapeDef skidShape = b3DefaultShapeDef();
			skidShape.density = 10.0f;
			skidShape.baseMaterial.friction = 0.7f;
			skidShape.filter.categoryBits = 0x0001;
			for ( float sx : { -0.7f, 0.7f } )
			{
				b3BoxHull skid = b3MakeOffsetBoxHull( 0.04f, 0.04f, 0.8f, { sx, -0.65f, 0.0f } );
				b3BoxHull strutF = b3MakeOffsetBoxHull( 0.03f, 0.35f, 0.03f, { sx * 0.7f, -0.35f, 0.4f } );
				b3BoxHull strutB = b3MakeOffsetBoxHull( 0.03f, 0.35f, 0.03f, { sx * 0.7f, -0.35f, -0.4f } );
				b3CreateHullShape( m_chassisId, &skidShape, &skid.base );
				b3CreateHullShape( m_chassisId, &skidShape, &strutF.base );
				b3CreateHullShape( m_chassisId, &skidShape, &strutB.base );
			}

			// 2. Lower Rotor Hub (Spins CW around +Y)
			b3Pos hub1Pos = { startPos.x, startPos.y + 0.55f, startPos.z };
			b3BodyDef hub1Def = b3DefaultBodyDef();
			hub1Def.type = b3_dynamicBody;
			hub1Def.allowFastRotation = true;
			hub1Def.position = hub1Pos;
			m_hub1Id = b3CreateBody( m_worldId, &hub1Def );

			b3ShapeDef hubShape = b3DefaultShapeDef();
			hubShape.density = 2.0f;
			hubShape.filter.categoryBits = 0x0002;
			hubShape.filter.maskBits = 0x0001;
			b3HullData* hubCyl1 = b3CreateCylinder( 0.08f, 0.25f, 0.0f, 16 );
			if ( hubCyl1 )
			{
				b3CreateHullShape( m_hub1Id, &hubShape, hubCyl1 );
				b3DestroyHull( hubCyl1 );
			}

			// Lower Blades (CW rotation)
			b3ShapeDef bladeShape = b3DefaultShapeDef();
			bladeShape.density = 0.35f;
			bladeShape.enableLift = true;
			bladeShape.filter.categoryBits = 0x0002;
			bladeShape.filter.maskBits = 0x0001;

			for ( int i = 0; i < m_bladeCount; ++i )
			{
				float az = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
				b3HullData* bHull = CreateRadialRotorBladeHull( 0.25f, m_propRadius, m_bladeChord, m_bladePitchDeg, az, false );
				if ( bHull )
				{
					b3CreateHullShape( m_hub1Id, &bladeShape, bHull );
					b3DestroyHull( bHull );
				}
			}

			// 3. Upper Rotor Hub (Spins CCW around +Y)
			b3Pos hub2Pos = { startPos.x, startPos.y + 0.85f, startPos.z };
			b3BodyDef hub2Def = b3DefaultBodyDef();
			hub2Def.type = b3_dynamicBody;
			hub2Def.allowFastRotation = true;
			hub2Def.position = hub2Pos;
			m_hub2Id = b3CreateBody( m_worldId, &hub2Def );

			b3HullData* hubCyl2 = b3CreateCylinder( 0.08f, 0.25f, 0.0f, 16 );
			if ( hubCyl2 )
			{
				b3CreateHullShape( m_hub2Id, &hubShape, hubCyl2 );
				b3DestroyHull( hubCyl2 );
			}

			// Upper Blades (CCW rotation with inverted tangent direction)
			for ( int i = 0; i < m_bladeCount; ++i )
			{
				float az = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
				b3HullData* bHull = CreateRadialRotorBladeHull( 0.25f, m_propRadius, m_bladeChord, m_bladePitchDeg, az, true );
				if ( bHull )
				{
					b3CreateHullShape( m_hub2Id, &bladeShape, bHull );
					b3DestroyHull( bHull );
				}
			}

			// Motor 1: Connect Chassis to Lower Hub (Revolute joint along vertical Y)
			b3RevoluteJointDef jDef1 = b3DefaultRevoluteJointDef();
			jDef1.base.bodyIdA = m_chassisId;
			jDef1.base.bodyIdB = m_hub1Id;
			jDef1.base.localFrameA.p = { 0.0f, 0.55f, 0.0f };
			jDef1.base.localFrameB.p = b3Vec3_zero;
			jDef1.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			jDef1.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			jDef1.enableMotor = true;
			jDef1.maxMotorTorque = 400.0f;
			jDef1.motorSpeed = GetCurrentRadianSpeed();
			m_motor1 = b3CreateRevoluteJoint( m_worldId, &jDef1 );

			// Motor 2: Connect Chassis to Upper Hub (Counter-Rotating Revolute joint)
			b3RevoluteJointDef jDef2 = b3DefaultRevoluteJointDef();
			jDef2.base.bodyIdA = m_chassisId;
			jDef2.base.bodyIdB = m_hub2Id;
			jDef2.base.localFrameA.p = { 0.0f, 0.85f, 0.0f };
			jDef2.base.localFrameB.p = b3Vec3_zero;
			jDef2.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			jDef2.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			jDef2.enableMotor = true;
			jDef2.maxMotorTorque = 400.0f;
			jDef2.motorSpeed = -GetCurrentRadianSpeed();
			m_motor2 = b3CreateRevoluteJoint( m_worldId, &jDef2 );
		}
		else if ( m_mode == 1 )
		{
			float railZ = -2.2f;

			// 1. Static base foundation & vertical guide rail
			b3BodyDef baseDef = b3DefaultBodyDef();
			baseDef.position = { 0.0f, 0.2f, railZ };
			b3BodyId baseId = b3CreateBody( m_worldId, &baseDef );
			b3BoxHull baseBox = b3MakeBoxHull( 0.6f, 0.2f, 0.6f );
			b3ShapeDef baseShape = b3DefaultShapeDef();
			b3CreateHullShape( baseId, &baseShape, &baseBox.base );

			// Static guide post (centered at railZ, extends to Y = 6.0m)
			b3BodyDef postDef = b3DefaultBodyDef();
			postDef.position = { 0.0f, 3.2f, railZ };
			b3BodyId postId = b3CreateBody( m_worldId, &postDef );
			b3Capsule postCap = { { 0.0f, -3.0f, 0.0f }, { 0.0f, 3.0f, 0.0f }, 0.08f };
			b3ShapeDef postShape = b3DefaultShapeDef();
			postShape.filter.categoryBits = 0x0001;
			b3CreateCapsuleShape( postId, &postShape, &postCap );

			// 2. Sliding Carriage with horizontal cantilever boom
			b3BodyDef carriageDef = b3DefaultBodyDef();
			carriageDef.type = b3_dynamicBody;
			carriageDef.position = { 0.0f, 1.2f, railZ };
			carriageDef.linearDamping = 0.2f;
			carriageDef.angularDamping = 0.8f;
			m_chassisId = b3CreateBody( m_worldId, &carriageDef );

			// Carriage block on the rail
			b3BoxHull carriageBox = b3MakeBoxHull( 0.25f, 0.3f, 0.25f );
			// Forward boom extending from railZ (-2.2m) to rotor origin (0.0m)
			b3BoxHull boomBox = b3MakeOffsetBoxHull( 0.06f, 0.06f, 1.1f, { 0.0f, 0.0f, 1.1f } );
			// Motor mount head at the forward tip (Z = 0.0m relative to carriage)
			b3BoxHull headBox = b3MakeOffsetBoxHull( 0.15f, 0.15f, 0.15f, { 0.0f, 0.15f, 2.2f } );

			b3ShapeDef carriageShape = b3DefaultShapeDef();
			carriageShape.density = 25.0f;
			carriageShape.filter.categoryBits = 0x0001;
			b3CreateHullShape( m_chassisId, &carriageShape, &carriageBox.base );
			b3CreateHullShape( m_chassisId, &carriageShape, &boomBox.base );
			b3CreateHullShape( m_chassisId, &carriageShape, &headBox.base );

			// Prismatic Joint along vertical Y axis
			b3PrismaticJointDef pDef = b3DefaultPrismaticJointDef();
			pDef.base.bodyIdA = postId;
			pDef.base.bodyIdB = m_chassisId;
			pDef.base.localFrameA.p = { 0.0f, -2.0f, 0.0f };
			pDef.base.localFrameB.p = b3Vec3_zero;
			// Rotate joint axis from X to Y
			pDef.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
			pDef.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
			pDef.enableLimit = true;
			pDef.lowerTranslation = -0.6f;
			pDef.upperTranslation = 4.5f;
			pDef.enableSpring = true;
			pDef.hertz = 3.0f;
			pDef.dampingRatio = 0.85f;
			pDef.targetTranslation = 0.0f;
			m_guideJoint = b3CreatePrismaticJoint( m_worldId, &pDef );

			// 3. Propeller Hub (at the forward tip Z = 0.0m, clear of guide post)
			b3Pos hubPos = { 0.0f, 1.6f, 0.0f };
			b3BodyDef hubDef = b3DefaultBodyDef();
			hubDef.type = b3_dynamicBody;
			hubDef.allowFastRotation = true;
			hubDef.position = hubPos;
			m_hub1Id = b3CreateBody( m_worldId, &hubDef );

			b3ShapeDef hubShape = b3DefaultShapeDef();
			hubShape.density = 2.0f;
			hubShape.filter.categoryBits = 0x0002;
			hubShape.filter.maskBits = 0x0001;
			b3HullData* hubCyl = b3CreateCylinder( 0.08f, 0.25f, 0.0f, 16 );
			if ( hubCyl )
			{
				b3CreateHullShape( m_hub1Id, &hubShape, hubCyl );
				b3DestroyHull( hubCyl );
			}

			// Blades (rotate around vertical Y axis at Z = 0.0m)
			b3ShapeDef bladeShape = b3DefaultShapeDef();
			bladeShape.density = 0.35f;
			bladeShape.enableLift = true;
			bladeShape.filter.categoryBits = 0x0002;
			bladeShape.filter.maskBits = 0x0001;

			for ( int i = 0; i < m_bladeCount; ++i )
			{
				float az = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
				b3HullData* bHull = CreateRadialRotorBladeHull( 0.25f, m_propRadius, m_bladeChord, m_bladePitchDeg, az, false );
				if ( bHull )
				{
					b3CreateHullShape( m_hub1Id, &bladeShape, bHull );
					b3DestroyHull( bHull );
				}
			}

			// Motor Revolute Joint connecting Carriage boom head (local {0, 0.4, 2.2}) to Hub
			b3RevoluteJointDef rDef = b3DefaultRevoluteJointDef();
			rDef.base.bodyIdA = m_chassisId;
			rDef.base.bodyIdB = m_hub1Id;
			rDef.base.localFrameA.p = { 0.0f, 0.4f, 2.2f };
			rDef.base.localFrameB.p = b3Vec3_zero;
			rDef.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			rDef.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			rDef.enableMotor = true;
			rDef.maxMotorTorque = 400.0f;
			rDef.motorSpeed = GetCurrentRadianSpeed();
			m_motor1 = b3CreateRevoluteJoint( m_worldId, &rDef );
		}
		else
		{
			b3Pos startPos = { 0.0f, 0.6f, 15.0f };

			// 1. Cart Chassis
			b3BodyDef cartDef = b3DefaultBodyDef();
			cartDef.type = b3_dynamicBody;
			cartDef.position = startPos;
			cartDef.linearDamping = 0.02f;
			cartDef.angularDamping = 0.5f;
			m_chassisId = b3CreateBody( m_worldId, &cartDef );

			// Floor frame
			b3BoxHull chassisBox = b3MakeBoxHull( 0.5f, 0.08f, 1.2f );
			// Engine pylon / firewall rising up to Y = 1.9m
			b3BoxHull pylonBox = b3MakeOffsetBoxHull( 0.12f, 0.55f, 0.12f, { 0.0f, 0.65f, -0.9f } );
			// Engine nacelle head
			b3BoxHull nacelleBox = b3MakeOffsetBoxHull( 0.18f, 0.18f, 0.25f, { 0.0f, 1.2f, -0.9f } );

			b3ShapeDef chassisShape = b3DefaultShapeDef();
			chassisShape.density = 25.0f;
			chassisShape.filter.categoryBits = 0x0001;
			b3CreateHullShape( m_chassisId, &chassisShape, &chassisBox.base );
			b3CreateHullShape( m_chassisId, &chassisShape, &pylonBox.base );
			b3CreateHullShape( m_chassisId, &chassisShape, &nacelleBox.base );

			// 2. 4 Rolling Wheels (Axle height Y = 0.35m, radius 0.35m, sits on ground Y = 0.0m)
			for ( float sx : { -0.65f, 0.65f } )
			{
				for ( float sz : { -0.8f, 0.8f } )
				{
					b3BodyDef wheelDef = b3DefaultBodyDef();
					wheelDef.type = b3_dynamicBody;
					wheelDef.allowFastRotation = true;
					wheelDef.position = { startPos.x + sx, 0.35f, startPos.z + sz };
					wheelDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
					wheelDef.linearDamping = 0.01f;
					wheelDef.angularDamping = 0.05f;
					b3BodyId wheelId = b3CreateBody( m_worldId, &wheelDef );

					b3Capsule wCap = { { -0.06f, 0.0f, 0.0f }, { 0.06f, 0.0f, 0.0f }, 0.35f };
					b3ShapeDef wShape = b3DefaultShapeDef();
					wShape.density = 15.0f;
					wShape.baseMaterial.friction = 0.8f;
					wShape.baseMaterial.rollingResistance = 0.002f;
					wShape.filter.categoryBits = 0x0001;
					b3CreateCapsuleShape( wheelId, &wShape, &wCap );

					b3RevoluteJointDef wJoint = b3DefaultRevoluteJointDef();
					wJoint.base.bodyIdA = m_chassisId;
					wJoint.base.bodyIdB = wheelId;
					wJoint.base.localFrameA.p = { sx, -0.25f, sz };
					wJoint.base.localFrameB.p = b3Vec3_zero;
					// Revolute joint axis along world X
					wJoint.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * B3_PI );
					wJoint.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * B3_PI );
					b3CreateRevoluteJoint( m_worldId, &wJoint );
				}
			}

			// 3. Forward Tractor Propeller Hub (Elevated at Y = 1.8m, well clear of ground)
			float propR = b3MinFloat( 1.1f, m_propRadius );
			b3Pos hubPos = { startPos.x, 1.8f, startPos.z - 1.25f };
			b3BodyDef hubDef = b3DefaultBodyDef();
			hubDef.type = b3_dynamicBody;
			hubDef.allowFastRotation = true;
			hubDef.position = hubPos;
			hubDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			m_hub1Id = b3CreateBody( m_worldId, &hubDef );

			b3ShapeDef hubShape = b3DefaultShapeDef();
			hubShape.density = 2.0f;
			hubShape.filter.categoryBits = 0x0002;
			hubShape.filter.maskBits = 0x0001;
			b3HullData* hubCyl = b3CreateCylinder( 0.08f, 0.25f, 0.0f, 16 );
			if ( hubCyl )
			{
				b3CreateHullShape( m_hub1Id, &hubShape, hubCyl );
				b3DestroyHull( hubCyl );
			}

			// Tractor Blades
			b3ShapeDef bladeShape = b3DefaultShapeDef();
			bladeShape.density = 0.35f;
			bladeShape.enableLift = true;
			bladeShape.filter.categoryBits = 0x0002;
			bladeShape.filter.maskBits = 0x0001;

			for ( int i = 0; i < m_bladeCount; ++i )
			{
				float az = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
				b3HullData* bHull = CreateRadialRotorBladeHull( 0.25f, propR, m_bladeChord, m_bladePitchDeg, az, false );
				if ( bHull )
				{
					b3CreateHullShape( m_hub1Id, &bladeShape, bHull );
					b3DestroyHull( bHull );
				}
			}

			// Motor along Z axis (spinning around world horizontal -Z axis)
			b3RevoluteJointDef rDef = b3DefaultRevoluteJointDef();
			rDef.base.bodyIdA = m_chassisId;
			rDef.base.bodyIdB = m_hub1Id;
			rDef.base.localFrameA.p = { 0.0f, 1.2f, -1.25f };
			rDef.base.localFrameB.p = b3Vec3_zero;
			rDef.base.localFrameA.q = b3Quat_identity;
			rDef.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
			rDef.enableMotor = true;
			rDef.maxMotorTorque = 400.0f;
			rDef.motorSpeed = -GetCurrentRadianSpeed();
			m_motor1 = b3CreateRevoluteJoint( m_worldId, &rDef );
		}
	}

	void Step() override
	{
		m_time += 1.0f / 60.0f;

		UpdateMotorSpeeds();

		// Apply steering trim torque in drone mode
		if ( m_mode == 0 && B3_IS_NON_NULL( m_chassisId ) )
		{
			b3Vec3 torque = { m_pitchTrim * 250.0f, 0.0f, -m_rollTrim * 250.0f };
			b3Body_ApplyTorque( m_chassisId, torque, true );
		}

		Sample::Step();

		// Telemetry & Visualizations
		float currentRPM = m_throttle * m_maxRPM;
		float omega = GetCurrentRadianSpeed();
		float tipSpeedMs = omega * m_propRadius;
		float tipSpeedKmh = tipSpeedMs * 3.6f;

		b3Pos chassisPos = B3_IS_NON_NULL( m_chassisId ) ? b3Body_GetPosition( m_chassisId ) : b3Pos_zero;
		b3Vec3 chassisVel = B3_IS_NON_NULL( m_chassisId ) ? b3Body_GetLinearVelocity( m_chassisId ) : b3Vec3_zero;

		// Draw Thrust Vector & Slipstream visualization
		if ( B3_IS_NON_NULL( m_hub1Id ) )
		{
			b3Pos hubPos = b3Body_GetPosition( m_hub1Id );
			b3Quat hubRot = b3Body_GetRotation( m_hub1Id );

			b3Vec3 thrustDir = ( m_mode == 2 ) ? b3Vec3{ 0.0f, 0.0f, -1.0f } : b3Vec3{ 0.0f, 1.0f, 0.0f };
			float arrowLen = b3MinFloat( 6.0f, 0.5f + 5.5f * m_throttle );
			b3Pos arrowTip = b3Add( hubPos, b3MulSV( arrowLen, thrustDir ) );
			Vec4 arrowColor = { 1.0f, 0.55f, 0.1f, 0.9f };
			DrawArrow( hubPos, arrowTip, arrowColor );

			// Draw spinning slipstream disk
			int segs = 20;
			float r = m_propRadius;
			for ( int i = 0; i < segs; ++i )
			{
				float th1 = (float)i * ( 2.0f * B3_PI / (float)segs );
				float th2 = (float)( i + 1 ) * ( 2.0f * B3_PI / (float)segs );

				b3Pos p1, p2;
				if ( m_mode == 2 )
				{
					p1 = { hubPos.x + r * cosf( th1 ), hubPos.y + r * sinf( th1 ), hubPos.z };
					p2 = { hubPos.x + r * cosf( th2 ), hubPos.y + r * sinf( th2 ), hubPos.z };
				}
				else
				{
					p1 = { hubPos.x + r * cosf( th1 ), hubPos.y, hubPos.z + r * sinf( th1 ) };
					p2 = { hubPos.x + r * cosf( th2 ), hubPos.y, hubPos.z + r * sinf( th2 ) };
				}

				Vec4 diskCol = { 0.3f, 0.8f, 1.0f, 0.4f * m_throttle };
				DrawLine( p1, p2, diskCol );
			}
		}

		DrawTextLine( "Aerodynamics: Propeller Thrust & Propulsion" );
		DrawTextLine( "Throttle: %.0f%% | Motor Speed: %.0f RPM | Blade Tip Speed: %.1f m/s (%.1f km/h)",
					  m_throttle * 100.0f, currentRPM, tipSpeedMs, tipSpeedKmh );
		DrawTextLine( "Propeller Geometry: %d Blades | Radius: %.2f m | Blade Pitch: %.1f deg",
					  m_bladeCount, m_propRadius, m_bladePitchDeg );
		DrawTextLine( "Vehicle Altitude: %.2f m | Vertical Velocity: %.2f m/s", chassisPos.y, chassisVel.y );
		DrawTextLine( "Controls: [W]/[S] Throttle Up/Down | [Space] Toggle 100%% Throttle | [R] Reset" );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS )
		{
			if ( key == KEY_W )
			{
				m_throttle = b3MinFloat( 1.0f, m_throttle + 0.05f );
			}
			else if ( key == KEY_S )
			{
				m_throttle = b3MaxFloat( 0.0f, m_throttle - 0.05f );
			}
			else if ( key == KEY_SPACE )
			{
				m_throttle = ( m_throttle > 0.1f ) ? 0.0f : 1.0f;
			}
			else if ( key == KEY_R )
			{
				BuildVehicle();
			}
		}
	}

	bool DrawControls() override
	{
		bool changed = false;

		const char* modes[] = { "Coaxial VTOL Drone (Free Flight)", "Thrust Test Stand (Vertical Rail)",
								"Runway Propeller Cart (Horizontal)" };
		if ( ImGui::Combo( "Demo Mode", &m_mode, modes, IM_ARRAYSIZE( modes ) ) )
		{
			BuildVehicle();
			changed = true;
		}

		ImGui::Separator();
		ImGui::SliderFloat( "Throttle", &m_throttle, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Max Motor RPM", &m_maxRPM, 500.0f, 4000.0f, "%.0f" );

		ImGui::Separator();
		if ( ImGui::SliderFloat( "Blade Pitch (deg)", &m_bladePitchDeg, -15.0f, 30.0f, "%.1f" ) )
		{
			BuildVehicle();
			changed = true;
		}
		if ( ImGui::SliderInt( "Blade Count", &m_bladeCount, 2, 6 ) )
		{
			BuildVehicle();
			changed = true;
		}
		if ( ImGui::SliderFloat( "Prop Radius (m)", &m_propRadius, 0.6f, 2.5f, "%.2f" ) )
		{
			BuildVehicle();
			changed = true;
		}
		if ( ImGui::SliderFloat( "Blade Chord (m)", &m_bladeChord, 0.10f, 0.40f, "%.2f" ) )
		{
			BuildVehicle();
			changed = true;
		}

		if ( m_mode == 0 )
		{
			ImGui::Separator();
			ImGui::Text( "Flight Attitude Trim:" );
			ImGui::SliderFloat( "Pitch Trim", &m_pitchTrim, -1.0f, 1.0f, "%.2f" );
			ImGui::SliderFloat( "Roll Trim", &m_rollTrim, -1.0f, 1.0f, "%.2f" );
			if ( ImGui::Button( "Level Attitude" ) )
			{
				m_pitchTrim = 0.0f;
				m_rollTrim = 0.0f;
			}
		}

		ImGui::Separator();
		if ( ImGui::Button( "Reset & Rebuild (R)" ) )
		{
			BuildVehicle();
			changed = true;
		}

		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new PropellerThrust( context );
	}

private:
	b3BodyId m_groundId;
	b3BodyId m_chassisId;
	b3BodyId m_hub1Id;
	b3BodyId m_hub2Id;
	b3JointId m_motor1;
	b3JointId m_motor2;
	b3JointId m_guideJoint;

	int m_mode;
	float m_throttle;
	float m_maxRPM;
	float m_bladePitchDeg;
	int m_bladeCount;
	float m_propRadius;
	float m_bladeChord;
	float m_payloadMass;
	float m_pitchTrim;
	float m_rollTrim;
	float m_time;
};

static int samplePropellerThrust = RegisterSample( "Aerodynamics", "Propeller Thrust", PropellerThrust::Create );





