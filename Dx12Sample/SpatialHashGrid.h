#pragma once
#include "pch.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "PhysicsObject.h"

struct XMF3Hash {
    size_t operator()(DirectX::XMFLOAT3 const& c) const noexcept {
        // Harvest the integer bits
        auto xi = static_cast<uint32_t>(c.x);
        auto yi = static_cast<uint32_t>(c.y);
        auto zi = static_cast<uint32_t>(c.z);
        // 3 large primes for mixing
        return (xi * 73856093u) ^ (yi * 19349663u) ^ (zi * 83492791u);
    }
};

struct XMF3Eq {
    bool operator()(DirectX::XMFLOAT3 const& a,
        DirectX::XMFLOAT3 const& b) const noexcept {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

struct PairKey {
    PhysicsObject* a; PhysicsObject* b;
    PairKey(PhysicsObject* x, PhysicsObject* y) {
        if (x < y) { a = x; b = y; }
        else { a = y; b = x; }
    }
    bool operator==(PairKey const& o) const {
        return a == o.a && b == o.b;
    }

    bool operator<(const PairKey& o) const noexcept
    {
        return (a < o.a) || (a == o.a && b < o.b);
    }
};

struct PairKeyHash {
    size_t operator()(PairKey const& k) const noexcept {
        return std::hash<PhysicsObject*>()(k.a)
            ^ (std::hash<PhysicsObject*>()(k.b) << 1);
    }
};

class SpatialHashGrid {
public:
    explicit SpatialHashGrid(float cellSize)
        : m_cellSize(cellSize) {
    }

    // called once at startup or whenever a new object appears
    void addBody(PhysicsObject* obj, AABB const& box, bool isStatic) {
        auto cellList = worldToCells(box);
        for (auto const& cell : cellList)
            m_cells[cell].push_back(obj);

        if (isStatic) {
            m_staticBodies.insert(obj);
        }
        else {
            m_dynamicBodies.insert(obj);
            m_bodyCells[obj] = std::move(cellList);
            m_prevAABBs[obj] = box;
        }
    }

	void clear() {
		m_cells.clear();
		m_dynamicBodies.clear();
		m_staticBodies.clear();
		m_bodyCells.clear();
		m_prevAABBs.clear();
	}

    // if you ever remove an object mid-game
    void removeBody(PhysicsObject* obj) {
        auto it = m_bodyCells.find(obj);
        if (it != m_bodyCells.end()) {
            // dynamic
            for (auto const& cell : it->second) {
                auto& vec = m_cells[cell];
                vec.erase(std::remove(vec.begin(), vec.end(), obj), vec.end());
            }
            m_bodyCells.erase(obj);
            m_prevAABBs.erase(obj);
            m_dynamicBodies.erase(obj);
        }
        else if (m_staticBodies.erase(obj)) {

        }
    }

    void updateDynamicBodies() {
        for (auto* obj : m_dynamicBodies) {
            AABB const& oldBox = m_prevAABBs[obj];
            AABB       newBox = obj->getCollider()->getWorldAABB(&obj->getTransform());

            auto oldCells = worldToCells(oldBox);
            auto newCells = worldToCells(newBox);

            auto cellsEqual = [&]() {
                if (oldCells.size() != newCells.size()) return false;
                for (auto const& c : oldCells) {
                    auto it = std::find_if(
                        newCells.begin(), newCells.end(),
                        [&](DirectX::XMFLOAT3 const& n) {
                            return n.x == c.x && n.y == c.y && n.z == c.z;
                        });
                    if (it == newCells.end()) return false;
                }
                return true;
                };
            if (cellsEqual()) continue;

            for (auto const& c : oldCells) {
                auto stillHere = std::find_if(
                    newCells.begin(), newCells.end(),
                    [&](DirectX::XMFLOAT3 const& n) {
                        return n.x == c.x && n.y == c.y && n.z == c.z;
                    });
                if (stillHere == newCells.end()) {
                    auto& vec = m_cells[c];
                    vec.erase(
                        std::remove_if(vec.begin(), vec.end(),
                            [&](PhysicsObject* o) { return o == obj; }),
                        vec.end()
                    );
                }
            }

            for (auto const& c : newCells) {
                auto wasHere = std::find_if(
                    oldCells.begin(), oldCells.end(),
                    [&](DirectX::XMFLOAT3 const& o) {
                        return o.x == c.x && o.y == c.y && o.z == c.z;
                    });
                if (wasHere == oldCells.end()) {
                    m_cells[c].push_back(obj);
                }
            }

            m_bodyCells[obj] = std::move(newCells);
            m_prevAABBs[obj] = newBox;
        }
    }

    std::vector<std::pair<PhysicsObject*, PhysicsObject*>>
        computePairs(bool skipStaticStatic = true) const {
        std::vector<std::pair<PhysicsObject*, PhysicsObject*>> result;
        std::unordered_set<PairKey, PairKeyHash> seen;

        for (auto const& kv : m_cells) {
            auto const& objs = kv.second;
            for (size_t i = 0; i < objs.size(); ++i) {
                for (size_t j = i + 1; j < objs.size(); ++j) {
                    auto* A = objs[i];
                    auto* B = objs[j];
                    if (skipStaticStatic && A->isStatic() && B->isStatic())
                        continue;
                    PairKey pk(A, B);
                    if (seen.insert(pk).second)
                        result.emplace_back(pk.a, pk.b);
                }
            }
        }
        return result;
    }

private:
    float m_cellSize;
    std::unordered_map<DirectX::XMFLOAT3, std::vector<PhysicsObject*>, XMF3Hash, XMF3Eq> m_cells;

    std::unordered_set<PhysicsObject*> m_dynamicBodies;
    std::unordered_set<PhysicsObject*> m_staticBodies;
    std::unordered_map<PhysicsObject*, std::vector<DirectX::XMFLOAT3>> m_bodyCells;
    std::unordered_map<PhysicsObject*, AABB>                         m_prevAABBs;

    std::vector<DirectX::XMFLOAT3> worldToCells(AABB const& box) const {
        auto floor3 = [&](float v) { return std::floor(v / m_cellSize); };
        int minX = int(floor3(box.getMinFloat().x)), minY = int(floor3(box.getMinFloat().y)), minZ = int(floor3(box.getMinFloat().z));
        int maxX = int(floor3(box.getMaxFloat().x)), maxY = int(floor3(box.getMaxFloat().y)), maxZ = int(floor3(box.getMaxFloat().z));
        std::vector<DirectX::XMFLOAT3> cells;
        for (int x = minX; x <= maxX; ++x)
            for (int y = minY; y <= maxY; ++y)
                for (int z = minZ; z <= maxZ; ++z)
                    cells.push_back({ float(x),float(y),float(z) });
        return cells;
    }
};