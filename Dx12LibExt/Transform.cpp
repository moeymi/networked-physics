#include "Transform.h"
#include "Helpers.h"

void Transform::SetParent(const std::shared_ptr<Transform>& newParent) {
	parent = newParent;
	CleanDirty();
}

bool Transform::IsStatic() const { return isStatic; }
void Transform::SetStatic(bool value) {
	isStatic = value;
	CleanDirty();
}

void Transform::CalculateWorldMatrix() {
	DirectX::XMMATRIX translation = DirectX::XMMatrixTranslationFromVector(position);
	DirectX::XMMATRIX rotation = DirectX::XMMatrixRotationQuaternion(rotationQuaternion);
	DirectX::XMMATRIX scaling = DirectX::XMMatrixScalingFromVector(scale);

	worldMatrix = scaling * rotation * translation;
}

void Transform::CalculateNewLookDirection() {
	lookDirection = DirectX::XMVector3Rotate(DirectX::XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f), GetRotationQuaternion());
	upDirection = DirectX::XMVector3Rotate(DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f), GetRotationQuaternion());
}

void Transform::CleanDirty() {
	if(positionScaleDirty || rotationDirty) CalculateWorldMatrix();
	if (rotationDirty) CalculateNewLookDirection();
	for (auto& listener : dirtyListeners) {
		listener();
	}

	positionScaleDirty = false;
	rotationDirty = false;
}

DirectX::XMMATRIX Transform::GetWorldMatrix() {
	if (positionScaleDirty || rotationDirty) {
		CleanDirty();
	}
	if (parent != nullptr) {
		return worldMatrix * parent->GetWorldMatrix();
	}
	return worldMatrix;
}

DirectX::XMVECTOR Transform::GetPosition() const {
	if (parent) {
		return DirectX::XMVector3Transform(position, parent->GetWorldMatrix());
	}
	return position;
}
void Transform::SetPosition(const DirectX::XMFLOAT3& pos) {
	if (isStatic) return;
	// Set position in local space from world space
	if (parent) {
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parent->GetWorldMatrix());
		position = DirectX::XMVector3Transform(DirectX::XMLoadFloat3(&pos), invParentWorld);
	}
	else {
		position = DirectX::XMLoadFloat3(&pos);
	}
	positionScaleDirty = true;
	CleanDirty();
}
void Transform::SetPosition(const DirectX::XMVECTOR& pos) {
	if (isStatic) return;
	// Set position in local space from world space
	if (parent) {
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parent->GetWorldMatrix());
		position = DirectX::XMVector3Transform(pos, invParentWorld);
	}
	else {
		position = pos;
	}

	positionScaleDirty = true;
	CleanDirty();
}

const DirectX::XMVECTOR& Transform::GetLocalPosition() const { return position; }
void Transform::SetLocalPosition(const DirectX::XMFLOAT3& pos) {
	if (isStatic) return;
	position = DirectX::XMLoadFloat3(&pos);

	positionScaleDirty = true;
	CleanDirty();
}
void Transform::SetLocalPosition(const DirectX::XMVECTOR& pos) {
	if (isStatic) return;
	position = pos;

	positionScaleDirty = true;
	CleanDirty();
}

DirectX::XMVECTOR Transform::GetScale() const {
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix();
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		return DirectX::XMVectorMultiply(scale, parentScale);
	}
	return scale;
}
void Transform::SetScale(const DirectX::XMFLOAT3& s) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix();
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		scale = DirectX::XMVectorDivide(DirectX::XMLoadFloat3(&s), parentScale);
	}
	else {
		scale = DirectX::XMLoadFloat3(&s);
	}

	positionScaleDirty = true;
	CleanDirty();
}
void Transform::SetScale(const DirectX::XMVECTOR& s) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMMATRIX parentWorld = parent->GetWorldMatrix();
		DirectX::XMMATRIX invParentWorld = DirectX::XMMatrixInverse(nullptr, parentWorld);
		DirectX::XMVECTOR parentScale = DirectX::XMVector3Length(invParentWorld.r[0]);
		scale = DirectX::XMVectorDivide(s, parentScale);
	}
	else {
		scale = s;
	}

	positionScaleDirty = true;
	CleanDirty();
}

const DirectX::XMVECTOR& Transform::GetLocalScale() const { return scale; }
void Transform::SetLocalScale(const DirectX::XMFLOAT3& s) {
	if (isStatic) return;
	scale = DirectX::XMLoadFloat3(&s);

	positionScaleDirty = true;
	CleanDirty();
}
void Transform::SetLocalScale(const DirectX::XMVECTOR& s) {
	if (isStatic) return;
	scale = s;

	positionScaleDirty = true;
	CleanDirty();
}


DirectX::XMVECTOR Transform::GetRotationQuaternion() const {
	if (parent != nullptr) {
		DirectX::XMVECTOR parentRot = parent->GetRotationQuaternion();
		return DirectX::XMQuaternionMultiply(rotationQuaternion, parentRot);
	}
	return rotationQuaternion;
}
void Transform::SetRotationQuaternion(const DirectX::XMVECTOR& rot) {
	if (isStatic) return;
	if (parent != nullptr) {
		DirectX::XMVECTOR parentRot = parent->GetRotationQuaternion();
		rotationQuaternion = DirectX::XMQuaternionMultiply(rot, DirectX::XMQuaternionInverse(parentRot));
	}
	else {
		rotationQuaternion = rot;
	}

	rotationDirty = true;
	CleanDirty();
}

const DirectX::XMVECTOR& Transform::GetLocalRotationQuaternion() const { return rotationQuaternion; }
void Transform::SetLocalRotationQuaternion(const DirectX::XMVECTOR& rot) {
	if (isStatic) return;
	rotationQuaternion = rot;
	CleanDirty();
}

DirectX::XMFLOAT3 Transform::GetRotationEulerAngles() const {
	if (parent != nullptr) {
		DirectX::XMFLOAT3 parentRot = parent->GetRotationEulerAngles();
		DirectX::XMFLOAT3 rot = Math::QuaternionToEuler(rotationQuaternion);
		return { rot.x + parentRot.x, rot.y + parentRot.y, rot.z + parentRot.z };
	}
	return Math::QuaternionToEuler(rotationQuaternion);
}
void Transform::SetRotationEulerAngles(const DirectX::XMFLOAT3& rot) {
	if (isStatic) return;
	if (parent) {
		DirectX::XMFLOAT3 parentRot = parent->GetRotationEulerAngles();
		rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x - parentRot.x, rot.y - parentRot.y, rot.z - parentRot.z);
	}
	else {
		rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x, rot.y, rot.z);
	}

	rotationDirty = true;
	CleanDirty();
}

DirectX::XMFLOAT3 Transform::GetLocalRotationEulerAngles() const {
	return Math::QuaternionToEuler(rotationQuaternion);
}

void Transform::SetLocalRotationEulerAngles(const DirectX::XMFLOAT3& rot) {
	if (isStatic) return;
	rotationQuaternion = DirectX::XMQuaternionRotationRollPitchYaw(rot.x, rot.y, rot.z);

	rotationDirty = true;
	CleanDirty();
}

const DirectX::XMVECTOR& Transform::GetLookDirection() const {
	return lookDirection;
}

const DirectX::XMVECTOR& Transform::GetUpDirection() const {
	return upDirection;
}

void Transform::Translate(const DirectX::XMFLOAT3& translation) {
	if (isStatic) return;
	position = DirectX::XMVectorAdd(position, DirectX::XMLoadFloat3(&translation));

	rotationDirty = true;
	CleanDirty();
}

void Transform::Translate(const DirectX::XMVECTOR& translation) {
	if (isStatic) return;
	position = DirectX::XMVectorAdd(position, translation);

	positionScaleDirty = true;
	CleanDirty();
}

void Transform::RotateEulerAngles(const DirectX::XMFLOAT3& rotation) {
	if (isStatic) return;
	DirectX::XMVECTOR rot = DirectX::XMQuaternionRotationRollPitchYaw(rotation.x, rotation.y, rotation.z);
	rotationQuaternion = DirectX::XMQuaternionMultiply(rotationQuaternion, rot);

	rotationDirty = true;
	CleanDirty();
}

void Transform::RotateQuaternion(const DirectX::XMVECTOR& rotation) {
	if (isStatic) return;
	rotationQuaternion = DirectX::XMQuaternionMultiply(rotationQuaternion, rotation);
	
	rotationDirty = true;
	CleanDirty();
}

void Transform::LookAt(const DirectX::XMVECTOR& target, const DirectX::XMVECTOR& up) {
	if (isStatic) return;
	DirectX::XMVECTOR forward = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(target, position));
	DirectX::XMVECTOR right = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(up, forward));
	DirectX::XMVECTOR newUp = DirectX::XMVector3Cross(forward, right);
	DirectX::XMMATRIX view = DirectX::XMMatrixLookToLH(position, forward, newUp);
	DirectX::XMMATRIX worldRot = DirectX::XMMatrixInverse(nullptr, view);
	rotationQuaternion = DirectX::XMQuaternionRotationMatrix(worldRot);
	
	rotationDirty = true;
	CleanDirty();
}