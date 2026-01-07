import SwiftUI
import MetalKit

struct MetalView: UIViewRepresentable {
    func makeUIView(context: Context) -> MTKView {
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal not supported on this device")
        }

        let view = MTKView(frame: .zero, device: device)
        view.clearColor = MTLClearColor(red: 0.06, green: 0.06, blue: 0.08, alpha: 1.0)
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.clearDepth = 1.0

        view.preferredFramesPerSecond = 60
        view.isPaused = false
        view.enableSetNeedsDisplay = false
        view.isMultipleTouchEnabled = true

        context.coordinator.configure(view: view)
        view.delegate = context.coordinator

        // ✅ Orbit rotate
        let pan = UIPanGestureRecognizer(target: context.coordinator, action: #selector(Renderer.handlePan(_:)))
        view.addGestureRecognizer(pan)

        // ✅ Zoom
        let pinch = UIPinchGestureRecognizer(target: context.coordinator, action: #selector(Renderer.handlePinch(_:)))
        view.addGestureRecognizer(pinch)

        // ✅ Reset (double tap)
        let dbl = UITapGestureRecognizer(target: context.coordinator, action: #selector(Renderer.handleDoubleTap(_:)))
        dbl.numberOfTapsRequired = 2
        view.addGestureRecognizer(dbl)

        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    func makeCoordinator() -> Renderer { Renderer() }
}
