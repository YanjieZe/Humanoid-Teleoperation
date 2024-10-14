import WebKit
import SwiftUI
import RealityKit
import ARKit

struct 🌐RealityView: View {
    var model: 🥽AppModel
    var body: some View {

        RealityView { content, attachments in

            // 创建并配置 x 轴实体
            let xAxisMesh = MeshResource.generateBox(size: [0.5, 0.01, 0.01])
            let xAxisMaterial = SimpleMaterial(color: .red, isMetallic: true)
            let xAxisEntity = ModelEntity(mesh: xAxisMesh, materials: [xAxisMaterial])
            xAxisEntity.position = [0.25, 0, 0] // 将其移到原点右侧
            // 创建并配置 y 轴实体
            let yAxisMesh = MeshResource.generateBox(size: [0.01, 0.01, 0.5])
            let yAxisMaterial = SimpleMaterial(color: .green, isMetallic: true)
            let yAxisEntity = ModelEntity(mesh: yAxisMesh, materials: [yAxisMaterial])
            yAxisEntity.position = [0, 0, -0.25] // 将其移到原点前方
            // 创建并配置 z 轴实体
            let zAxisMesh = MeshResource.generateBox(size: [0.01, 0.5, 0.01])
            let zAxisMaterial = SimpleMaterial(color: .blue, isMetallic: true)
            let zAxisEntity = ModelEntity(mesh: zAxisMesh, materials: [zAxisMaterial])
            zAxisEntity.position = [0, 0.25, 0] // 将其移到原点上方
            // 将坐标轴实体添加到场景内容
            content.add(xAxisEntity)
            content.add(yAxisEntity)
            content.add(zAxisEntity)


            let resultLabelEntity = attachments.entity(for: Self.attachmentID)!
            resultLabelEntity.components.set(🧑HeadTrackingComponent())
            resultLabelEntity.name = 🧩Name.resultLabel

            // 获取并配置 webViewEntity
            let webViewEntity = attachments.entity(for: Self.webViewAttachmentID)!
            webViewEntity.position = [0, 1.0, -1.5] // 将其放置在视野前方

            // content.add(webViewEntity)
            content.add(resultLabelEntity)

        } attachments: {
            Attachment(id: Self.attachmentID) {
                 WebView(url: URL(string: "https://192.168.31.157:8012/?ws=wss://192.168.31.157:8012")!)
                .frame(width: 1500, height: 1200)
            }
            Attachment(id: Self.webViewAttachmentID) {
                // WebView(url: URL(string: "https://192.168.31.157:8012/?ws=wss://192.168.31.157:8012")!)
                // .frame(width: 1500, height: 1200)
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
        )
        .task { self.model.run() }
        .task { await self.model.processDeviceAnchorUpdates() }
        .task { self.model.startserver() }
        .task(priority: .low) { await self.model.processReconstructionUpdates() }
        // 这几个task才是核心发送信息的东西，RealityView无关紧要

    }
    static let attachmentID: String = "resultLabel"
    static let webViewAttachmentID: String = "webViewAttachment"

}

struct WebView: UIViewRepresentable {
    var url: URL

    func makeUIView(context: Context) -> some UIView {
        let configuration = WKWebViewConfiguration()
        configuration.allowsPictureInPictureMediaPlayback = true
        configuration.allowsInlinePredictions = true
        configuration.allowsInlineMediaPlayback = true
        configuration.allowsAirPlayForMediaPlayback = true
        
        let webView = WKWebView(frame: .zero, configuration: configuration)
        webView.load(URLRequest(url: url))
        
        return webView
    }
    
    func updateUIView(_ uiView: UIViewType, context: Context) {
        
    }
}


