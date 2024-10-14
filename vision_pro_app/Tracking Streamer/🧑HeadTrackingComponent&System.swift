import RealityKit
import ARKit
import SwiftUI

struct 🧑HeadTrackingComponent: Component, Codable {
    init() {}
}

struct 🧑HeadTrackingSystem: System {
    private static let query = EntityQuery(where: .has(🧑HeadTrackingComponent.self))
    
    private let session = ARKitSession()
    private let provider = WorldTrackingProvider()
    
    init(scene: RealityKit.Scene) {
        self.setUpSession()
    }
    
    private func setUpSession() {
        Task {
            do {
                try await self.session.run([self.provider])
            } catch {
                assertionFailure()
            }
        }
    }
    
    func update(context: SceneUpdateContext) {
        let entities = context.scene.performQuery(Self.query).map { $0 }
        
        guard !entities.isEmpty,
                let deviceAnchor = self.provider.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else { return }
        
        let cameraTransform = Transform(matrix: deviceAnchor.originFromAnchorTransform)

        // 计算前方1米的位置
        let forwardVector = cameraTransform.matrix.columns.2 // Z方向向量
        let forwardOffset = simd_make_float3(forwardVector.x, forwardVector.y, forwardVector.z) * -1.0 // 取反方向
        let targetPosition = cameraTransform.translation + forwardOffset // 前方1米位置

        
        for entity in entities {
            // 设置实体的新位置
            entity.setPosition(targetPosition, relativeTo: nil)
            
            // 使实体朝向摄像头位置
            entity.look(at: cameraTransform.translation,
                    from: targetPosition,
                    relativeTo: nil,
                    forward: .positiveZ)
            
            // entity.look(at: cameraTransform.translation,
            //             from: entity.position(relativeTo: nil),
            //             relativeTo: nil,
            //             forward: .positiveZ)
        }
    }
}


