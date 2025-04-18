#include "Transform.h"
#include "Helpers.h"

void Transform::SetParent(const std::shared_ptr<Transform>& newParent) {
	parent = newParent;
	CleanDirty(0, true);
}

bool Transform::IsStatic() const { return isStatic; }
void Transform::SetStatic(bool value) {
	isStatic = value;
	CleanDirty(0, true);
}

void Transform::swapStates() {
	m_states[0] = m_states[1];
}

void Transform::CalculateWorldMatrix(const int& bufferIndex, const bool& bothBuffers) {
	DirectX::XMMATRIX translation = DirectX::XMMatrixTranslationFromVector(m_states[bufferIndex].position);
	DirectX::XMMATRIX rotation = DirectX::XMMatrixRotationQuaternion(m_states[bufferIndex].rotationQuaternion);
	DirectX::XMMATRIX scaling = DirectX::XMMatrixScalingFromVector(m_states[bufferIndex].scale);

	m_states[bufferIndex].worldMatrix = scaling * rotation * translation;

	if (bothBuffers) {
		CalculateWorldMatrix((bufferIndex + 1) % 2, false);
	}
}

void Transform::CalculateNewLookDirection(const int& bufferIndex, const bool& bothBuffers) {
	m_states[bufferIndex].lookDirection = DirectX::XMVector3Rotate(DirectX::XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f), GetRotationQuaternion(bufferIndex));
	m_states[bufferIndex].upDirection = DirectX::XMVector3Rotate(DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f), GetRotationQuaternion(bufferIndex));

	if (bothBuffers) {
		CalculateNewLookDirection((bufferIndex + 1) % 2, false);
	}
}

void Transform::CleanDirty(const int& bufferIndex, const bool& bothBuffers) {
	if(m_states[bufferIndex].positionScaleDirty || m_states[bufferIndex].rotationDirty) CalculateWorldMatrix(bufferIndex, false);
	if (m_states[bufferIndex].rotationDirty) CalculateNewLookDirection(bufferIndex, false);
	for (auto& listener : dirtyListeners) {
		listener();
	}

	m_states[bufferIndex].positionScaleDirty = false;
	m_states[bufferIndex].rotationDirty = false;

	if (bothBuffers) {
		CleanDirty((bufferIndex + 1) % 2, false);
	}
}

DirectX::XMMATRIX Transform::GetWorldMatrix(const int& bufferIndex) {
	if (m_states[bufferIndex].positionScaleDirty || m_states[bufferIndex].rotationDirty) {
		CleanDirty(bufferIndex, false);
	}
	if (parent != nullptr) {
		return m_states[bufferIndex].worldMatrix * parent->GetWorldMatrix(bufferIndex);
	}
	return m_states[bufferIndex].worldMatrix;
}

DirectX::XMVECTOR Transform::GetPosition(const int& bufferIndex) const {
	if (parent) {
		return DirectX::XMVector3Transform(m_states[bufferIndex].position, parent->GetWorldMatrix(bufferIndex));
	}
	return m_states[bufferIndex].position;
}

void Transform::SetPosition(const DirectX::XMFLOAT3& pos, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	// Set position in local space from world space
	if (parent) {
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parent->GetWorldMatrix(bufferIndex));
		m_states[bufferIndex].position = DirectX::XMVector3Transform(DirectX::XMLoadFloat3(&pos), invParentWorld);
	}
	else {
		m_states[bufferIndex].position = DirectX::XMLoadFloat3(&pos);
	}
	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetPosition(pos, (bufferIndex + 1) % 2, false);
	}
}
void Transform::SetPosition(const DirectX::XMVECTOR& pos, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	// Set position in local space from world space
	if (parent) {
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parent->GetWorldMatrix(bufferIndex));
		m_states[bufferIndex].position = DirectX::XMVector3Transform(pos, invParentWorld);
	}
	else {
		m_states[bufferIndex].position = pos;
	}

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetPosition(pos, (bufferIndex + 1) % 2, false);
	}
}

const DirectX::XMVECTOR& Transform::GetLocalPosition(const int& bufferIndex) const { return m_states[bufferIndex].position; }
void Transform::SetLocalPosition(const DirectX::XMFLOAT3& pos, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].position = DirectX::XMLoadFloat3(&pos);

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetLocalPosition(pos, (bufferIndex + 1) % 2, false);
	}
}
void Transform::SetLocalPosition(const DirectX::XMVECTOR& pos, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].position = pos;

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetLocalPosition(pos, (bufferIndex + 1) % 2, false);
	}
}

DirectX::XMVECTOR Transform::GetScale(const int& bufferIndex) const {
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix(bufferIndex);
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		return DirectX::XMVectorMultiply(m_states[bufferIndex].scale, parentScale);
	}
	return m_states[bufferIndex].scale;
}
void Transform::SetScale(const DirectX::XMFLOAT3& s, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix(bufferIndex);
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		m_states[bufferIndex].scale = DirectX::XMVectorDivide(DirectX::XMLoadFloat3(&s), parentScale);
	}
	else {
		m_states[bufferIndex].scale = DirectX::XMLoadFloat3(&s);
	}

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetScale(s, (bufferIndex + 1) % 2, false);
	}
}
void Transform::SetScale(const DirectX::XMVECTOR& s, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix(bufferIndex);
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		m_states[bufferIndex].scale = DirectX::XMVectorDivide(s, parentScale);
	}
	else {
		m_states[bufferIndex].scale = s;
	}

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetScale(s, (bufferIndex + 1) % 2, false);
	}
}

const DirectX::XMVECTOR& Transform::GetLocalScale(const int& bufferIndex) const { return m_states[bufferIndex].scale; }
void Transform::SetLocalScale(const DirectX::XMFLOAT3& s, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].scale = DirectX::XMLoadFloat3(&s);

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetLocalScale(s, (bufferIndex + 1) % 2, false);
	}
}
void Transform::SetLocalScale(const DirectX::XMVECTOR& s, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].scale = s;

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);
}


DirectX::XMVECTOR Transform::GetRotationQuaternion(const int& bufferIndex) const {
	if (parent != nullptr) {
		DirectX::XMVECTOR parentRot = parent->GetRotationQuaternion(bufferIndex);
		return DirectX::XMQuaternionMultiply(m_states[bufferIndex].rotationQuaternion, parentRot);
	}
	return m_states[bufferIndex].rotationQuaternion;
}
void Transform::SetRotationQuaternion(const DirectX::XMVECTOR& rot, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	if (parent != nullptr) {
		DirectX::XMVECTOR parentRot = parent->GetRotationQuaternion(bufferIndex);
		m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionMultiply(rot, DirectX::XMQuaternionInverse(parentRot));
	}
	else {
		m_states[bufferIndex].rotationQuaternion = rot;
	}

	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetRotationQuaternion(rot, (bufferIndex + 1) % 2, false);
	}
}

const DirectX::XMVECTOR& Transform::GetLocalRotationQuaternion(const int& bufferIndex) const { return m_states[bufferIndex].rotationQuaternion; }
void Transform::SetLocalRotationQuaternion(const DirectX::XMVECTOR& rot, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].rotationQuaternion = rot;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetLocalRotationQuaternion(rot, (bufferIndex + 1) % 2, false);
	}
}

DirectX::XMFLOAT3 Transform::GetRotationEulerAngles(const int& bufferIndex) const {
	if (parent != nullptr) {
		DirectX::XMFLOAT3 parentRot = parent->GetRotationEulerAngles(bufferIndex);
		DirectX::XMFLOAT3 rot = Math::QuaternionToEuler(m_states[bufferIndex].rotationQuaternion);
		return { rot.x + parentRot.x, rot.y + parentRot.y, rot.z + parentRot.z };
	}
	return Math::QuaternionToEuler(m_states[bufferIndex].rotationQuaternion);
}
void Transform::SetRotationEulerAngles(const DirectX::XMFLOAT3& rot, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMFLOAT3 parentRot = parent->GetRotationEulerAngles(bufferIndex);
		m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x - parentRot.x, rot.y - parentRot.y, rot.z - parentRot.z);
	}
	else {
		m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x, rot.y, rot.z);
	}

	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetRotationEulerAngles(rot, (bufferIndex + 1) % 2, false);
	}
}

DirectX::XMFLOAT3 Transform::GetLocalRotationEulerAngles(const int& bufferIndex) const {
	return Math::QuaternionToEuler(m_states[bufferIndex].rotationQuaternion);
}

void Transform::SetLocalRotationEulerAngles(const DirectX::XMFLOAT3& rot, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x, rot.y, rot.z);

	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		SetLocalRotationEulerAngles(rot, (bufferIndex + 1) % 2, false);
	}
}

const DirectX::XMVECTOR& Transform::GetLookDirection(const int& bufferIndex) const {
	return m_states[bufferIndex].lookDirection;
}

const DirectX::XMVECTOR& Transform::GetUpDirection(const int& bufferIndex) const {
	return m_states[bufferIndex].upDirection;
}

void Transform::Translate(const DirectX::XMFLOAT3& translation, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].position = DirectX::XMVectorAdd(m_states[bufferIndex].position, DirectX::XMLoadFloat3(&translation));

	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		Translate(translation, (bufferIndex + 1) % 2, false);
	}
}

void Transform::Translate(const DirectX::XMVECTOR& translation, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].position = DirectX::XMVectorAdd(m_states[bufferIndex].position, translation);

	m_states[bufferIndex].positionScaleDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		Translate(translation, (bufferIndex + 1) % 2, false);
	}
}

void Transform::RotateEulerAngles(const DirectX::XMFLOAT3& rotation, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	DirectX::XMVECTOR rot = DirectX::XMQuaternionRotationRollPitchYaw(rotation.x, rotation.y, rotation.z);
	m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionMultiply(m_states[bufferIndex].rotationQuaternion, rot);

	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		RotateEulerAngles(rotation, (bufferIndex + 1) % 2, false);
	}
}

void Transform::RotateQuaternion(const DirectX::XMVECTOR& rotation, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionMultiply(m_states[bufferIndex].rotationQuaternion, rotation);
	
	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		RotateQuaternion(rotation, (bufferIndex + 1) % 2, false);
	}
}

void Transform::LookAt(const DirectX::XMVECTOR& target, const DirectX::XMVECTOR& up, const int& bufferIndex, const bool& bothBuffers) {
	if (isStatic) return;
	DirectX::XMVECTOR forward = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(target, m_states[bufferIndex].position));
	DirectX::XMVECTOR right = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(up, forward));
	DirectX::XMVECTOR newUp = DirectX::XMVector3Cross(forward, right);
	DirectX::XMMATRIX view = DirectX::XMMatrixLookToLH(m_states[bufferIndex].position, forward, newUp);
	DirectX::XMMATRIX worldRot = DirectX::XMMatrixInverse(nullptr, view);
	m_states[bufferIndex].rotationQuaternion = DirectX::XMQuaternionRotationMatrix(worldRot);
	
	m_states[bufferIndex].rotationDirty = true;
	CleanDirty(bufferIndex, false);

	if (bothBuffers) {
		LookAt(target, up, (bufferIndex + 1) % 2, false);
	}
}