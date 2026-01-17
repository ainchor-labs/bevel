/**
 * Jolt Physics Python bindings for Bevel engine.
 *
 * This module provides Python bindings for the Jolt Physics library
 * using pybind11.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include <unordered_map>
#include <mutex>
#include <cstdarg>

namespace py = pybind11;

// Jolt's trace callback
static void TraceImpl(const char* inFMT, ...) {
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    // Could pipe to Python logging
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char* inExpression, const char* inMessage,
                              const char* inFile, uint32_t inLine) {
    // Return true to trigger debugger break
    return false;
}
#endif

// Layers for collision filtering
namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

// Broad phase layers
namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr uint32_t NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    virtual uint32_t GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch ((JPH::BroadPhaseLayer::Type)inLayer) {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING: return "MOVING";
            default: JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

std::vector<uint32_t> query_aabb(
    std::tuple<float, float, float> min_point,
    std::tuple<float, float, float> max_point
) {
    std::vector<uint32_t> results;
    if (!m_initialized) return results;
    
    JPH::AABox box(
        JPH::Vec3(std::get<0>(min_point), std::get<1>(min_point), std::get<2>(min_point)),
        JPH::Vec3(std::get<0>(max_point), std::get<1>(max_point), std::get<2>(max_point))
    );
    
    class MyCollector : public JPH::CollideShapeBodyCollector {
    public:
        std::vector<uint32_t>& results;
        const std::unordered_map<uint32_t, JPH::BodyID>& body_map;
        
        MyCollector(std::vector<uint32_t>& r, const std::unordered_map<uint32_t, JPH::BodyID>& bm)
            : results(r), body_map(bm) {}
        
        void AddHit(const JPH::BodyID& inBodyID) override {
            for (const auto& [id, body_id] : body_map) {
                if (body_id == inBodyID) {
                    results.push_back(id);
                    break;
                }
            }
        }
    };
    
    MyCollector collector(results, m_body_map);
    m_physics_system->GetBroadPhaseQuery().CollideAABox(box, collector);
    
    return results;
}

std::vector<uint32_t> query_sphere(
    std::tuple<float, float, float> center,
    float radius
) {
    std::vector<uint32_t> results;
    if (!m_initialized) return results;
    
    JPH::RVec3 sphere_center(std::get<0>(center), std::get<1>(center), std::get<2>(center));
    
    // Use AABB approximation for sphere
    JPH::Vec3 extent(radius, radius, radius);
    JPH::AABox box(sphere_center - extent, sphere_center + extent);
    
    class MyCollector : public JPH::CollideShapeBodyCollector {
    public:
        std::vector<uint32_t>& results;
        const std::unordered_map<uint32_t, JPH::BodyID>& body_map;
        JPH::RVec3 center;
        float radius_sq;
        JPH::PhysicsSystem* system;
        
        MyCollector(std::vector<uint32_t>& r, const std::unordered_map<uint32_t, JPH::BodyID>& bm,
                   JPH::RVec3 c, float r, JPH::PhysicsSystem* s)
            : results(r), body_map(bm), center(c), radius_sq(r * r), system(s) {}
        
        void AddHit(const JPH::BodyID& inBodyID) override {
            // Check actual distance
            JPH::RVec3 body_pos = system->GetBodyInterface().GetCenterOfMassPosition(inBodyID);
            if ((body_pos - center).LengthSq() <= radius_sq) {
                for (const auto& [id, body_id] : body_map) {
                    if (body_id == inBodyID) {
                        results.push_back(id);
                        break;
                    }
                }
            }
        }
    };
    
    MyCollector collector(results, m_body_map, sphere_center, radius, m_physics_system.get());
    m_physics_system->GetBroadPhaseQuery().CollideAABox(box, collector);
    
    return results;
}

// ObjectVsBroadPhaseLayerFilter implementation
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING:
                return inLayer2 == BroadPhaseLayers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// ObjectLayerPairFilter implementation
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        switch (inObject1) {
            case Layers::NON_MOVING:
                return inObject2 == Layers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// Contact listener for collision callbacks
class MyContactListener : public JPH::ContactListener {
public:
    py::function callback;
    std::mutex callback_mutex;

    virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2,
                                                   JPH::RVec3Arg inBaseOffset,
                                                   const JPH::CollideShapeResult& inCollisionResult) override {
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2,
                                const JPH::ContactManifold& inManifold,
                                JPH::ContactSettings& ioSettings) override {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (callback) {
            try {
                py::gil_scoped_acquire acquire;
                auto point = inManifold.GetWorldSpaceContactPointOn1(0);
                callback(
                    inBody1.GetID().GetIndexAndSequenceNumber(),
                    inBody2.GetID().GetIndexAndSequenceNumber(),
                    std::make_tuple(point.GetX(), point.GetY(), point.GetZ()),
                    std::make_tuple(inManifold.mWorldSpaceNormal.GetX(),
                                    inManifold.mWorldSpaceNormal.GetY(),
                                    inManifold.mWorldSpaceNormal.GetZ()),
                    inManifold.mPenetrationDepth
                );
            } catch (const py::error_already_set& e) {
                // Ignore Python errors in callback
            }
        }
    }
};

// Body activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener {
public:
    virtual void OnBodyActivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override {}
    virtual void OnBodyDeactivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override {}
};

/**
 * Main Jolt Physics World wrapper class.
 */
class JoltPhysicsWorld {
public:
    JoltPhysicsWorld()
        : m_initialized(false)
        , m_accumulator(0.0f)
        , m_next_body_id(1) {
    }

    ~JoltPhysicsWorld() {
        shutdown();
    }

    bool initialize() {
        if (m_initialized) return true;

        // Register Jolt allocator and trace
        JPH::RegisterDefaultAllocator();
        JPH::Trace = TraceImpl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

        // Create factory
        JPH::Factory::sInstance = new JPH::Factory();

        // Register physics types
        JPH::RegisterTypes();

        // Allocators
        m_temp_allocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
        m_job_system = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
            std::thread::hardware_concurrency() - 1);

        // Create physics system
        const uint32_t cMaxBodies = 65536;
        const uint32_t cNumBodyMutexes = 0;  // Default
        const uint32_t cMaxBodyPairs = 65536;
        const uint32_t cMaxContactConstraints = 10240;

        m_broad_phase_layer_interface = std::make_unique<BPLayerInterfaceImpl>();
        m_object_vs_broadphase_layer_filter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
        m_object_layer_pair_filter = std::make_unique<ObjectLayerPairFilterImpl>();

        m_physics_system = std::make_unique<JPH::PhysicsSystem>();
        m_physics_system->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
                               *m_broad_phase_layer_interface,
                               *m_object_vs_broadphase_layer_filter,
                               *m_object_layer_pair_filter);

        // Set up listeners
        m_contact_listener = std::make_unique<MyContactListener>();
        m_physics_system->SetContactListener(m_contact_listener.get());

        m_body_activation_listener = std::make_unique<MyBodyActivationListener>();
        m_physics_system->SetBodyActivationListener(m_body_activation_listener.get());

        // Default gravity
        m_physics_system->SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

        m_initialized = true;
        return true;
    }

    void shutdown() {
        if (!m_initialized) return;


        // Remove all constraints
        for (auto& [id, constraint] : m_constraint_map) {
            m_physics_system->RemoveConstraint(constraint);
            constraint->Release();
        }
        m_constraint_map.clear();

        // Remove all bodies
        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        for (auto& [id, body_id] : m_body_map) {
            body_interface.RemoveBody(body_id);
            body_interface.DestroyBody(body_id);
        }
        m_body_map.clear();

        m_physics_system.reset();
        m_job_system.reset();
        m_temp_allocator.reset();
        m_contact_listener.reset();
        m_body_activation_listener.reset();
        m_broad_phase_layer_interface.reset();
        m_object_vs_broadphase_layer_filter.reset();
        m_object_layer_pair_filter.reset();

        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;

        m_initialized = false;
    }

    void step(float delta_time) {
        if (!m_initialized) return;

        const float cFixedTimeStep = 1.0f / 60.0f;
        const int cCollisionSteps = 1;

        m_accumulator += delta_time;

        while (m_accumulator >= cFixedTimeStep) {
            m_physics_system->Update(cFixedTimeStep, cCollisionSteps,
                                     m_temp_allocator.get(), m_job_system.get());
            m_accumulator -= cFixedTimeStep;
        }
    }

    uint32_t add_box_body(
        std::tuple<float, float, float> position,
        std::tuple<float, float, float, float> rotation,
        std::tuple<float, float, float> half_extents,
        int body_type,
        float mass,
        float friction,
        float restitution
    ) {
        if (!m_initialized) return 0;

        JPH::BoxShapeSettings shape_settings(
            JPH::Vec3(std::get<0>(half_extents), std::get<1>(half_extents), std::get<2>(half_extents))
        );
        shape_settings.SetEmbedded();

        auto shape_result = shape_settings.Create();
        if (shape_result.HasError()) return 0;

        JPH::EMotionType motion_type;
        JPH::ObjectLayer layer;
        switch (body_type) {
            case 0: motion_type = JPH::EMotionType::Static; layer = Layers::NON_MOVING; break;
            case 1: motion_type = JPH::EMotionType::Kinematic; layer = Layers::MOVING; break;
            default: motion_type = JPH::EMotionType::Dynamic; layer = Layers::MOVING; break;
        }

        JPH::BodyCreationSettings body_settings(
            shape_result.Get(),
            JPH::RVec3(std::get<0>(position), std::get<1>(position), std::get<2>(position)),
            JPH::Quat(std::get<0>(rotation), std::get<1>(rotation),
                      std::get<2>(rotation), std::get<3>(rotation)),
            motion_type,
            layer
        );

        if (motion_type == JPH::EMotionType::Dynamic) {
            body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            body_settings.mMassPropertiesOverride.mMass = mass;
        }
        body_settings.mFriction = friction;
        body_settings.mRestitution = restitution;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        JPH::Body* body = body_interface.CreateBody(body_settings);
        if (body == nullptr) return 0;

        body_interface.AddBody(body->GetID(), JPH::EActivation::Activate);

        uint32_t id = m_next_body_id++;
        m_body_map[id] = body->GetID();
        return id;
    }

    uint32_t add_sphere_body(
        std::tuple<float, float, float> position,
        float radius,
        int body_type,
        float mass
    ) {
        if (!m_initialized) return 0;

        JPH::SphereShapeSettings shape_settings(radius);
        shape_settings.SetEmbedded();

        auto shape_result = shape_settings.Create();
        if (shape_result.HasError()) return 0;

        JPH::EMotionType motion_type;
        JPH::ObjectLayer layer;
        switch (body_type) {
            case 0: motion_type = JPH::EMotionType::Static; layer = Layers::NON_MOVING; break;
            case 1: motion_type = JPH::EMotionType::Kinematic; layer = Layers::MOVING; break;
            default: motion_type = JPH::EMotionType::Dynamic; layer = Layers::MOVING; break;
        }

        JPH::BodyCreationSettings body_settings(
            shape_result.Get(),
            JPH::RVec3(std::get<0>(position), std::get<1>(position), std::get<2>(position)),
            JPH::Quat::sIdentity(),
            motion_type,
            layer
        );

        if (motion_type == JPH::EMotionType::Dynamic) {
            body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            body_settings.mMassPropertiesOverride.mMass = mass;
        }

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        JPH::Body* body = body_interface.CreateBody(body_settings);
        if (body == nullptr) return 0;

        body_interface.AddBody(body->GetID(), JPH::EActivation::Activate);

        uint32_t id = m_next_body_id++;
        m_body_map[id] = body->GetID();
        return id;
    }

    uint32_t add_capsule_body(
        std::tuple<float, float, float> position,
        std::tuple<float, float, float, float> rotation,
        float radius,
        float half_height,
        int body_type,
        float mass
    ) {
        if (!m_initialized) return 0;

        JPH::CapsuleShapeSettings shape_settings(half_height, radius);
        shape_settings.SetEmbedded();

        auto shape_result = shape_settings.Create();
        if (shape_result.HasError()) return 0;

        JPH::EMotionType motion_type;
        JPH::ObjectLayer layer;
        switch (body_type) {
            case 0: motion_type = JPH::EMotionType::Static; layer = Layers::NON_MOVING; break;
            case 1: motion_type = JPH::EMotionType::Kinematic; layer = Layers::MOVING; break;
            default: motion_type = JPH::EMotionType::Dynamic; layer = Layers::MOVING; break;
        }

        JPH::BodyCreationSettings body_settings(
            shape_result.Get(),
            JPH::RVec3(std::get<0>(position), std::get<1>(position), std::get<2>(position)),
            JPH::Quat(std::get<0>(rotation), std::get<1>(rotation),
                      std::get<2>(rotation), std::get<3>(rotation)),
            motion_type,
            layer
        );

        if (motion_type == JPH::EMotionType::Dynamic) {
            body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            body_settings.mMassPropertiesOverride.mMass = mass;
        }

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        JPH::Body* body = body_interface.CreateBody(body_settings);
        if (body == nullptr) return 0;

        body_interface.AddBody(body->GetID(), JPH::EActivation::Activate);

        uint32_t id = m_next_body_id++;
        m_body_map[id] = body->GetID();
        return id;
    }

    void remove_body(uint32_t body_id) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.RemoveBody(it->second);
        body_interface.DestroyBody(it->second);
        m_body_map.erase(it);
    }

    std::tuple<float, float, float> get_body_position(uint32_t body_id) {
        if (!m_initialized) return {0, 0, 0};

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return {0, 0, 0};

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        auto pos = body_interface.GetCenterOfMassPosition(it->second);
        return {pos.GetX(), pos.GetY(), pos.GetZ()};
    }

    std::tuple<float, float, float, float> get_body_rotation(uint32_t body_id) {
        if (!m_initialized) return {0, 0, 0, 1};

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return {0, 0, 0, 1};

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        auto rot = body_interface.GetRotation(it->second);
        return {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
    }

    void set_body_position(uint32_t body_id, std::tuple<float, float, float> pos) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.SetPosition(it->second,
            JPH::RVec3(std::get<0>(pos), std::get<1>(pos), std::get<2>(pos)),
            JPH::EActivation::Activate);
    }

    void set_body_rotation(uint32_t body_id, std::tuple<float, float, float, float> rot) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.SetRotation(it->second,
            JPH::Quat(std::get<0>(rot), std::get<1>(rot), std::get<2>(rot), std::get<3>(rot)),
            JPH::EActivation::Activate);
    }

    void set_linear_velocity(uint32_t body_id, std::tuple<float, float, float> vel) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.SetLinearVelocity(it->second,
            JPH::Vec3(std::get<0>(vel), std::get<1>(vel), std::get<2>(vel)));
    }

    std::tuple<float, float, float> get_linear_velocity(uint32_t body_id) {
        if (!m_initialized) return {0, 0, 0};

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return {0, 0, 0};

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        auto vel = body_interface.GetLinearVelocity(it->second);
        return {vel.GetX(), vel.GetY(), vel.GetZ()};
    }

    void set_angular_velocity(uint32_t body_id, std::tuple<float, float, float> vel) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.SetAngularVelocity(it->second,
            JPH::Vec3(std::get<0>(vel), std::get<1>(vel), std::get<2>(vel)));
    }

    std::tuple<float, float, float> get_angular_velocity(uint32_t body_id) {
        if (!m_initialized) return {0, 0, 0};

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return {0, 0, 0};

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        auto vel = body_interface.GetAngularVelocity(it->second);
        return {vel.GetX(), vel.GetY(), vel.GetZ()};
    }

    void apply_force(uint32_t body_id, std::tuple<float, float, float> force) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.AddForce(it->second,
            JPH::Vec3(std::get<0>(force), std::get<1>(force), std::get<2>(force)));
    }

    void apply_impulse(uint32_t body_id, std::tuple<float, float, float> impulse) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.AddImpulse(it->second,
            JPH::Vec3(std::get<0>(impulse), std::get<1>(impulse), std::get<2>(impulse)));
    }

    void apply_torque(uint32_t body_id, std::tuple<float, float, float> torque) {
        if (!m_initialized) return;

        auto it = m_body_map.find(body_id);
        if (it == m_body_map.end()) return;

        JPH::BodyInterface& body_interface = m_physics_system->GetBodyInterface();
        body_interface.AddTorque(it->second,
            JPH::Vec3(std::get<0>(torque), std::get<1>(torque), std::get<2>(torque)));
    }

    void set_gravity(std::tuple<float, float, float> gravity) {
        if (!m_initialized) return;
        m_physics_system->SetGravity(
            JPH::Vec3(std::get<0>(gravity), std::get<1>(gravity), std::get<2>(gravity)));
    }

    std::tuple<float, float, float> get_gravity() {
        if (!m_initialized) return {0, -9.81f, 0};
        auto g = m_physics_system->GetGravity();
        return {g.GetX(), g.GetY(), g.GetZ()};
    }

    void set_collision_callback(py::function callback) {
        if (m_contact_listener) {
            std::lock_guard<std::mutex> lock(m_contact_listener->callback_mutex);
            m_contact_listener->callback = callback;
        }
    }

    std::vector<std::tuple<uint32_t, std::tuple<float, float, float>, std::tuple<float, float, float>>>
    raycast(std::tuple<float, float, float> start, std::tuple<float, float, float> end) {
        std::vector<std::tuple<uint32_t, std::tuple<float, float, float>, std::tuple<float, float, float>>> results;

        if (!m_initialized) return results;

        JPH::RRayCast ray;
        ray.mOrigin = JPH::RVec3(std::get<0>(start), std::get<1>(start), std::get<2>(start));
        JPH::Vec3 direction = JPH::Vec3(
            std::get<0>(end) - std::get<0>(start),
            std::get<1>(end) - std::get<1>(start),
            std::get<2>(end) - std::get<2>(start)
        );
        ray.mDirection = direction;

        class MyRayCastCollector : public JPH::CastRayCollector {
        public:
            std::vector<std::tuple<uint32_t, std::tuple<float, float, float>, std::tuple<float, float, float>>>& results;
            const std::unordered_map<uint32_t, JPH::BodyID>& body_map;

            MyRayCastCollector(
                std::vector<std::tuple<uint32_t, std::tuple<float, float, float>, std::tuple<float, float, float>>>& r,
                const std::unordered_map<uint32_t, JPH::BodyID>& bm
            ) : results(r), body_map(bm) {}

            void AddHit(const JPH::RayCastResult& inResult) override {
                // Find body ID
                for (const auto& [id, body_id] : body_map) {
                    if (body_id == inResult.mBodyID) {
                        auto point = inResult.mFraction * ray.mDirection + ray.mOrigin;
                        results.push_back(std::make_tuple(
                            id,
                            std::make_tuple(point.GetX(), point.GetY(), point.GetZ()),
                            std::make_tuple(
                                inResult.mSurfaceNormal.GetX(),
                                inResult.mSurfaceNormal.GetY(),
                                inResult.mSurfaceNormal.GetZ()
                            )
                        ));
                        break;
                    }
                }
            }
        };

        MyRayCastCollector collector(results, m_body_map);
        m_physics_system->GetNarrowPhaseQuery().CastRay(ray, collector);

        return results;
    }

    uint32_t create_fixed_joint(
        uint32_t body_a_id,
        uint32_t body_b_id,
        std::tuple<float, float, float> anchor_point
    ) {
        if (!m_initialized) return 0;
        
        auto it_a = m_body_map.find(body_a_id);
        auto it_b = m_body_map.find(body_b_id);
        if (it_a == m_body_map.end() || it_b == m_body_map.end()) return 0;
        
        JPH::FixedConstraintSettings settings;
        settings.mAutoDetectPoint = false;
        settings.mPoint1 = settings.mPoint2 = JPH::RVec3(
            std::get<0>(anchor_point),
            std::get<1>(anchor_point),
            std::get<2>(anchor_point)
        );
        
        JPH::Constraint* constraint = settings.Create(*m_physics_system->GetBodyInterface().GetBody(it_a->second),
                                                       *m_physics_system->GetBodyInterface().GetBody(it_b->second));
        if (!constraint) return 0;
        
        m_physics_system->AddConstraint(constraint);
        
        uint32_t id = m_next_constraint_id++;
        m_constraint_map[id] = constraint;
        return id;
    }
    
    uint32_t create_hinge_joint(
        uint32_t body_a_id,
        uint32_t body_b_id,
        std::tuple<float, float, float> anchor_point,
        std::tuple<float, float, float> axis,
        float min_angle,
        float max_angle
    ) {
        if (!m_initialized) return 0;
        
        auto it_a = m_body_map.find(body_a_id);
        auto it_b = m_body_map.find(body_b_id);
        if (it_a == m_body_map.end() || it_b == m_body_map.end()) return 0;
        
        JPH::HingeConstraintSettings settings;
        settings.mPoint1 = settings.mPoint2 = JPH::RVec3(
            std::get<0>(anchor_point),
            std::get<1>(anchor_point),
            std::get<2>(anchor_point)
        );
        settings.mHingeAxis1 = settings.mHingeAxis2 = JPH::Vec3(
            std::get<0>(axis),
            std::get<1>(axis),
            std::get<2>(axis)
        );
        settings.mNormalAxis1 = settings.mNormalAxis2 = JPH::Vec3::sAxisX();
        
        if (min_angle < max_angle) {
            settings.mLimitsMin = min_angle;
            settings.mLimitsMax = max_angle;
        }
        
        JPH::Constraint* constraint = settings.Create(*m_physics_system->GetBodyInterface().GetBody(it_a->second),
                                                       *m_physics_system->GetBodyInterface().GetBody(it_b->second));
        if (!constraint) return 0;
        
        m_physics_system->AddConstraint(constraint);
        
        uint32_t id = m_next_constraint_id++;
        m_constraint_map[id] = constraint;
        return id;
    }
    
    uint32_t create_slider_joint(
        uint32_t body_a_id,
        uint32_t body_b_id,
        std::tuple<float, float, float> anchor_point,
        std::tuple<float, float, float> axis,
        float min_distance,
        float max_distance
    ) {
        if (!m_initialized) return 0;
        
        auto it_a = m_body_map.find(body_a_id);
        auto it_b = m_body_map.find(body_b_id);
        if (it_a == m_body_map.end() || it_b == m_body_map.end()) return 0;
        
        JPH::SliderConstraintSettings settings;
        settings.mAutoDetectPoint = false;
        settings.mPoint1 = settings.mPoint2 = JPH::RVec3(
            std::get<0>(anchor_point),
            std::get<1>(anchor_point),
            std::get<2>(anchor_point)
        );
        settings.mSliderAxis1 = settings.mSliderAxis2 = JPH::Vec3(
            std::get<0>(axis),
            std::get<1>(axis),
            std::get<2>(axis)
        );
        
        if (min_distance < max_distance) {
            settings.mLimitsMin = min_distance;
            settings.mLimitsMax = max_distance;
        }
        
        JPH::Constraint* constraint = settings.Create(*m_physics_system->GetBodyInterface().GetBody(it_a->second),
                                                       *m_physics_system->GetBodyInterface().GetBody(it_b->second));
        if (!constraint) return 0;
        
        m_physics_system->AddConstraint(constraint);
        
        uint32_t id = m_next_constraint_id++;
        m_constraint_map[id] = constraint;
        return id;
    }
    
    uint32_t create_distance_joint(
        uint32_t body_a_id,
        uint32_t body_b_id,
        std::tuple<float, float, float> anchor_a,
        std::tuple<float, float, float> anchor_b,
        float min_distance,
        float max_distance
    ) {
        if (!m_initialized) return 0;
        
        auto it_a = m_body_map.find(body_a_id);
        auto it_b = m_body_map.find(body_b_id);
        if (it_a == m_body_map.end() || it_b == m_body_map.end()) return 0;
        
        JPH::DistanceConstraintSettings settings;
        settings.mPoint1 = JPH::RVec3(std::get<0>(anchor_a), std::get<1>(anchor_a), std::get<2>(anchor_a));
        settings.mPoint2 = JPH::RVec3(std::get<0>(anchor_b), std::get<1>(anchor_b), std::get<2>(anchor_b));
        settings.mMinDistance = min_distance;
        settings.mMaxDistance = max_distance;
        
        JPH::Constraint* constraint = settings.Create(*m_physics_system->GetBodyInterface().GetBody(it_a->second),
                                                       *m_physics_system->GetBodyInterface().GetBody(it_b->second));
        if (!constraint) return 0;
        
        m_physics_system->AddConstraint(constraint);
        
        uint32_t id = m_next_constraint_id++;
        m_constraint_map[id] = constraint;
        return id;
    }
    
    uint32_t create_point_joint(
        uint32_t body_a_id,
        uint32_t body_b_id,
        std::tuple<float, float, float> anchor_point
    ) {
        if (!m_initialized) return 0;
        
        auto it_a = m_body_map.find(body_a_id);
        auto it_b = m_body_map.find(body_b_id);
        if (it_a == m_body_map.end() || it_b == m_body_map.end()) return 0;
        
        JPH::PointConstraintSettings settings;
        settings.mPoint1 = settings.mPoint2 = JPH::RVec3(
            std::get<0>(anchor_point),
            std::get<1>(anchor_point),
            std::get<2>(anchor_point)
        );
        
        JPH::Constraint* constraint = settings.Create(*m_physics_system->GetBodyInterface().GetBody(it_a->second),
                                                       *m_physics_system->GetBodyInterface().GetBody(it_b->second));
        if (!constraint) return 0;
        
        m_physics_system->AddConstraint(constraint);
        
        uint32_t id = m_next_constraint_id++;
        m_constraint_map[id] = constraint;
        return id;
    }
    
    void destroy_joint(uint32_t joint_id) {
        auto it = m_constraint_map.find(joint_id);
        if (it == m_constraint_map.end()) return;
        
        m_physics_system->RemoveConstraint(it->second);
        it->second->Release();
        m_constraint_map.erase(it);
    }

private:
    bool m_initialized;
    float m_accumulator;
    uint32_t m_next_body_id;

    std::unique_ptr<JPH::TempAllocatorImpl> m_temp_allocator;
    std::unique_ptr<JPH::JobSystemThreadPool> m_job_system;
    std::unique_ptr<JPH::PhysicsSystem> m_physics_system;
    std::unique_ptr<BPLayerInterfaceImpl> m_broad_phase_layer_interface;
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterImpl> m_object_vs_broadphase_layer_filter;
    std::unique_ptr<ObjectLayerPairFilterImpl> m_object_layer_pair_filter;
    std::unique_ptr<MyContactListener> m_contact_listener;
    std::unique_ptr<MyBodyActivationListener> m_body_activation_listener;

    std::unordered_map<uint32_t, JPH::BodyID> m_body_map;
    std::unordered_map<uint32_t, JPH::Constraint*> m_constraint_map;
    uint32_t m_next_constraint_id = 1;
};

PYBIND11_MODULE(_jolt, m) {
    m.doc() = "Jolt Physics bindings for Bevel engine";

    py::class_<JoltPhysicsWorld>(m, "JoltPhysicsWorld")
        .def(py::init<>())
        .def("initialize", &JoltPhysicsWorld::initialize)
        .def("shutdown", &JoltPhysicsWorld::shutdown)
        .def("step", &JoltPhysicsWorld::step)
        .def("add_box_body", &JoltPhysicsWorld::add_box_body,
             py::arg("position"), py::arg("rotation"), py::arg("half_extents"),
             py::arg("body_type"), py::arg("mass"), py::arg("friction"), py::arg("restitution"))
        .def("add_sphere_body", &JoltPhysicsWorld::add_sphere_body,
             py::arg("position"), py::arg("radius"), py::arg("body_type"), py::arg("mass"))
        .def("add_capsule_body", &JoltPhysicsWorld::add_capsule_body,
             py::arg("position"), py::arg("rotation"), py::arg("radius"),
             py::arg("half_height"), py::arg("body_type"), py::arg("mass"))
        .def("remove_body", &JoltPhysicsWorld::remove_body)
        .def("get_body_position", &JoltPhysicsWorld::get_body_position)
        .def("get_body_rotation", &JoltPhysicsWorld::get_body_rotation)
        .def("set_body_position", &JoltPhysicsWorld::set_body_position)
        .def("set_body_rotation", &JoltPhysicsWorld::set_body_rotation)
        .def("set_linear_velocity", &JoltPhysicsWorld::set_linear_velocity)
        .def("get_linear_velocity", &JoltPhysicsWorld::get_linear_velocity)
        .def("set_angular_velocity", &JoltPhysicsWorld::set_angular_velocity)
        .def("get_angular_velocity", &JoltPhysicsWorld::get_angular_velocity)
        .def("apply_force", &JoltPhysicsWorld::apply_force)
        .def("apply_impulse", &JoltPhysicsWorld::apply_impulse)
        .def("apply_torque", &JoltPhysicsWorld::apply_torque)
        .def("set_gravity", &JoltPhysicsWorld::set_gravity)
        .def("get_gravity", &JoltPhysicsWorld::get_gravity)
        .def("set_collision_callback", &JoltPhysicsWorld::set_collision_callback)
        .def("raycast", &JoltPhysicsWorld::raycast);
        .def("create_fixed_joint", &JoltPhysicsWorld::create_fixed_joint)
        .def("create_hinge_joint", &JoltPhysicsWorld::create_hinge_joint)
        .def("create_slider_joint", &JoltPhysicsWorld::create_slider_joint)
        .def("create_distance_joint", &JoltPhysicsWorld::create_distance_joint)
        .def("create_point_joint", &JoltPhysicsWorld::create_point_joint)
        .def("destroy_joint", &JoltPhysicsWorld::destroy_joint)
        .def("query_aabb", &JoltPhysicsWorld::query_aabb)
        .def("query_sphere", &JoltPhysicsWorld::query_sphere);
}
