#pragma once
#include "pch.h"
#include "PhysicsObject.h"

#include <deque>
#include <memory>
#include <algorithm>

struct ObjectUpdate {
    uint32_t object_id;
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT4 rotation;
    DirectX::XMFLOAT3 velocity;
    DirectX::XMFLOAT3 angular_velocity;
};

struct Snapshot {
	uint64_t tick;
	double send_time;
	std::vector<ObjectUpdate> updates;
};

struct GhostState {
	uint64_t tick;
	ObjectUpdate update;
};

class NetworkedObject
{
private:
	uint16_t                       m_id;
	uint16_t                       m_ownerId;
	uint16_t                       m_lastAckedTick;

	std::shared_ptr<PhysicsObject> m_object;

	std::deque<GhostState>         m_buffer;
    ObjectUpdate                   m_lastSnapshot;
    uint64_t                       m_lastSnapshotTick = 0;
	bool						   m_received = false;
	float                          m_blendAlpha = 1;

    void updateGhost(const uint64_t& localTick, const int& delayTicks, const float& dt) {
        using namespace DirectX;
        GhostState  aState{
			0, ObjectUpdate{ 0, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0} }
		}, bState{
			0, ObjectUpdate{ 0, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0} }
		};
        bool        haveA = false, haveB = false;
        int64_t     target = int64_t(localTick) - delayTicks;

        // clamp target into the buffer’s span
        if (!m_buffer.empty())
        {
            target = std::clamp(target,
                int64_t(m_buffer.front().tick),
                int64_t(m_buffer.back().tick));
        }

        // try to find two samples to interpolate…
        for (size_t i = 0; i + 1 < m_buffer.size(); ++i)
        {
            auto& s0 = m_buffer[i];
            auto& s1 = m_buffer[i + 1];
            if (s0.tick <= target && target <= s1.tick)
            {
                aState = s0;  bState = s1;
                haveA = haveB = true;
                break;
            }
        }
        // if we only have one sample, extrapolate from it
        if (!haveA && m_buffer.size() == 1)
        {
            aState = m_buffer.front();
            haveA = true;
        }
        // if we have none, fall back to lastSnapshot
        if (!haveA && m_buffer.empty())
        {
            aState.tick = m_lastSnapshotTick;
            aState.update = m_lastSnapshot;
            haveA = true;
        }
        {
            uint64_t pruneUpTo = aState.tick;
            while (!m_buffer.empty() && m_buffer.front().tick <= pruneUpTo)
                m_buffer.pop_front();
        }

        XMFLOAT3 newPos, newVel, newAngVel;
        XMFLOAT4 newRot;

        if (haveA && haveB)
        {
            float denominator = float(bState.tick - aState.tick);
            float alpha = (denominator == 0.0f) ? 0.0f : float(target - aState.tick) / denominator;

            XMStoreFloat3(&newPos,
                XMVectorLerp(XMLoadFloat3(&aState.update.position),
                    XMLoadFloat3(&bState.update.position),
                    alpha));
            XMStoreFloat4(&newRot,
                XMQuaternionSlerp(XMLoadFloat4(&aState.update.rotation),
                    XMLoadFloat4(&bState.update.rotation),
                    alpha));
            XMStoreFloat3(&newVel,
                XMVectorLerp(XMLoadFloat3(&aState.update.velocity),
                    XMLoadFloat3(&bState.update.velocity),
                    alpha));
            XMStoreFloat3(&newAngVel,
                XMVectorLerp(XMLoadFloat3(&aState.update.angular_velocity),
                    XMLoadFloat3(&bState.update.angular_velocity),
                    alpha));
        }
        else
        {
            // read the pose you just displayed in the previous frame
            XMVECTOR curP = m_object->getTransform().GetPosition(0);
            XMVECTOR curR = m_object->getTransform().GetRotationQuaternion(0);

            // --- 2. Advance position & orientation by velocity * dt ----------
            XMVECTOR vLin = XMLoadFloat3(&aState.update.velocity);
            XMVECTOR vAng = XMLoadFloat3(&aState.update.angular_velocity);

            XMVECTOR  nextP = curP + vLin * dt;
            XMVECTOR  nextR = integrateRotation(curR, aState.update.angular_velocity, dt);

            XMStoreFloat3(&newPos, nextP);
            XMStoreFloat4(&newRot, nextR);

            // velocities stay the same
            newVel = aState.update.velocity;
            newAngVel = aState.update.angular_velocity;
        }

        auto& xf = m_object->getTransform();
        XMVECTOR errP = DirectX::XMVectorSubtract(XMLoadFloat3(&newPos), xf.GetPosition(0));

        m_blendAlpha = std::clamp(m_blendAlpha + dt * 1.0f, 0.0f, 1.0f);
        XMVECTOR blendedP = DirectX::XMVectorAdd(xf.GetPosition(0), DirectX::XMVectorScale(errP, m_blendAlpha));
        XMVECTOR blendedR = XMQuaternionSlerp(xf.GetRotationQuaternion(0),
            XMLoadFloat4(&newRot),
            m_blendAlpha);

        XMFLOAT3 blendedPFloat3;
		XMStoreFloat3(&blendedPFloat3, blendedP);

		// Check for any NAN values in the blended position or rotation
        if (std::isnan(blendedPFloat3.x) || std::isnan(blendedPFloat3.y) || std::isnan(blendedPFloat3.z)) {
			Log::Info() << "NetworkedObject: Last snapshot position contains NaN values at tick " << m_lastSnapshotTick << ", resetting to zero position.";
           // return;
        }


        xf.SetPosition(blendedPFloat3, 1);
        xf.SetRotationQuaternion(blendedR, 1);
        m_object->setVelocity(newVel, 1);
        m_object->setAngularVelocity(newAngVel, 1);
    }

    void snapToLast()
    {
        if (!m_object) {
			Log::Warn() << "NetworkedObject: No object set for snapping." << std::endl;
			return;
        }
		if (!m_received) {
			Log::Warn() << "NetworkedObject: No last snapshot available to snap to." << std::endl;
			return;
		}
        auto& xf = m_object->getTransform();

        Log::Info() << "NetworkedObject: Snapping to last snapshot at tick " << m_lastSnapshotTick << " for object ID " << m_id << " owned by " << m_ownerId << " with position "
            << m_lastSnapshot.position.x << ", " << m_lastSnapshot.position.y << ", " << m_lastSnapshot.position.z << std::endl;

        // Check for any NAN values
		if (std::isnan(m_lastSnapshot.position.x) || std::isnan(m_lastSnapshot.position.y) || std::isnan(m_lastSnapshot.position.z)) {
			Log::Info() << "NetworkedObject: Last snapshot position contains NaN values at tick " << m_lastSnapshotTick << ", resetting to zero position." << std::endl;
			m_lastSnapshot.position = { 0.0f, 0.0f, 0.0f };
		}

        xf.SetPosition(m_lastSnapshot.position, 1);
        xf.SetRotationQuaternion(m_lastSnapshot.rotation, 1);
        m_object->setVelocity(m_lastSnapshot.velocity, 1);
        m_object->setAngularVelocity(m_lastSnapshot.angular_velocity, 1);

        m_blendAlpha = 1.0f;
        m_buffer.clear();
    }

    DirectX::XMVECTOR integrateRotation(const DirectX::XMVECTOR& rotation, const DirectX::XMFLOAT3& angularVelocity, float dt) {
        using namespace DirectX;

        // Convert angular velocity (in radians/sec) to a quaternion derivative
        XMVECTOR omegaVec = XMVectorSet(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f);

        // qDot = 0.5 * q * omega (in quaternion form)
        XMVECTOR qDot = XMQuaternionMultiply(rotation, omegaVec);
        qDot = XMVectorScale(qDot, 0.5f * dt);

        // Integrate by adding qDot to current rotation
        XMVECTOR integrated = XMVectorAdd(rotation, qDot);

        // Normalize to maintain valid quaternion
        return XMQuaternionNormalize(integrated);
    }

public:
	NetworkedObject(const uint16_t& id, const uint16_t& ownerId, const std::shared_ptr<PhysicsObject>& object) : m_id(id), m_ownerId(ownerId), m_lastAckedTick(0),
		m_lastSnapshot({ 0, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0} })
    {
		setObject(object);
	}

	uint16_t getId() const { return m_id; }
	uint16_t getOwnerId() const { return m_ownerId; }
	uint16_t getLastAckedTick() const { return m_lastAckedTick; }
	void setLastAckedTick(uint16_t tick) { m_lastAckedTick = tick; }
	const std::shared_ptr<PhysicsObject>& getObject() const { return m_object; }

	void setObject(const std::shared_ptr<PhysicsObject>& object) {
		m_object = object;
	}

	bool buildUpdate(const uint64_t& tick, ObjectUpdate& out) {
		if (m_object) {
			out.object_id = m_id;
			DirectX::XMStoreFloat3(&out.position, m_object->getTransform().GetPosition(0));
			DirectX::XMStoreFloat4(&out.rotation, m_object->getTransform().GetRotationQuaternion(0));
			DirectX::XMStoreFloat3(&out.velocity, m_object->getVelocity(0));
			DirectX::XMStoreFloat3(&out.angular_velocity, m_object->getAngularVelocity(0));
			return true;
		}
		return false;
	}

    void applyUpdate(const uint64_t& tick, const int& delayTicks, const float& dt, const bool& ghostMode) {
        if (ghostMode) {
            updateGhost(tick, delayTicks, dt);
        }
        else {
            snapToLast();
        }
    }

	void addUpdate(const uint64_t& tick, const ObjectUpdate& update) {
        m_lastSnapshotTick = tick;
        m_lastSnapshot = update;
		m_received = true;

		m_buffer.push_back({ tick, update });
		if (m_buffer.size() > 4) {
			m_buffer.pop_front();
		}
	}
};