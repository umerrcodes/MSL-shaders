import Foundation
import Metal
import MetalKit
import simd



private var orientation = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))
private var angularVelocity = SIMD3<Float>(0, 0, 0)   // radians/sec around x/y/z
private var isInteracting = false

private var distance: Float = 2.6
private var lastFrameTime = CACurrentMediaTime()

private var lastPanTranslation = CGPoint.zero


final class Renderer: NSObject, MTKViewDelegate {
    private var device: MTLDevice!
    private var queue: MTLCommandQueue!
    private var pipeline: MTLRenderPipelineState!
    private var depthState: MTLDepthStencilState!

    private var vertexBuffer: MTLBuffer!
    private var startTime = CACurrentMediaTime()
    
    private func rotationMatrix(from q: simd_quatf) -> simd_float4x4 {
        let x = q.imag.x, y = q.imag.y, z = q.imag.z, w = q.real

        let m00: Float = 1 - 2*y*y - 2*z*z
        let m01: Float = 2*x*y - 2*z*w
        let m02: Float = 2*x*z + 2*y*w

        let m10: Float = 2*x*y + 2*z*w
        let m11: Float = 1 - 2*x*x - 2*z*z
        let m12: Float = 2*y*z - 2*x*w

        let m20: Float = 2*x*z - 2*y*w
        let m21: Float = 2*y*z + 2*x*w
        let m22: Float = 1 - 2*x*x - 2*y*y

        return simd_float4x4(columns: (
            SIMD4<Float>(m00, m10, m20, 0),
            SIMD4<Float>(m01, m11, m21, 0),
            SIMD4<Float>(m02, m12, m22, 0),
            SIMD4<Float>(0,   0,   0,   1)
        ))
    }


    // MARK: - Types

    private struct Vertex {
        var position: SIMD3<Float>
        var normal: SIMD3<Float>
    }

    private struct Uniforms {
        var mvp: simd_float4x4
        var model: simd_float4x4
        var normalMatrix: simd_float3x3
        var lightDir: SIMD3<Float>
        var cameraPos: SIMD3<Float>
        var _pad: Float = 0
        
        @objc func handlePan(_ g: UIPanGestureRecognizer) {
            guard let v = g.view else { return }

            switch g.state {
            case .began:
                isInteracting = true
                angularVelocity = .zero
                lastPanTranslation = g.translation(in: v)

            case .changed:
                let t = g.translation(in: v)
                let dx = Float(t.x - lastPanTranslation.x)
                let dy = Float(t.y - lastPanTranslation.y)
                lastPanTranslation = t

                // Trackball-ish: axis from drag direction (dy affects X rotation, dx affects Y rotation)
                let axis = simd_normalize(SIMD3<Float>(-dy, dx, 0))
                let angle = simd_length(SIMD2<Float>(dx, dy)) * 0.008  // sensitivity
                if angle.isFinite {
                    orientation = simd_quatf(angle: angle, axis: axis) * orientation
                    orientation = simd_normalize(orientation)
                }

            case .ended, .cancelled, .failed:
                isInteracting = false
                let vel = g.velocity(in: v) // pts/sec

                // Convert swipe velocity into angular velocity (tweak factor for feel)
                angularVelocity = SIMD3<Float>(
                    Float(-vel.y) * 0.00004,
                    Float( vel.x) * 0.00004,
                    0
                )

            default:
                break
            }
        }

        @objc func handlePinch(_ g: UIPinchGestureRecognizer) {
            // Make pinch smooth by consuming scale
            let s = Float(g.scale)
            g.scale = 1.0

            // Zoom in/out
            distance = max(1.2, min(8.0, distance / max(0.01, s)))
        }

        @objc func handleDoubleTap(_ g: UITapGestureRecognizer) {
            orientation = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))
            angularVelocity = .zero
            distance = 2.6
        }

    }

    // MARK: - Setup

    func configure(view: MTKView) {
        guard let device = view.device else { return }
        self.device = device

        queue = device.makeCommandQueue()

        // Build 3D extruded triangle (triangular prism)
        vertexBuffer = makePrismVertexBuffer(device: device)

        // Pipeline
        let library = device.makeDefaultLibrary()
        let v = library?.makeFunction(name: "vertex_main")
        let f = library?.makeFunction(name: "fragment_main")

        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction = v
        desc.fragmentFunction = f
        desc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        desc.depthAttachmentPixelFormat = view.depthStencilPixelFormat

        pipeline = try! device.makeRenderPipelineState(descriptor: desc)

        // Depth state
        let d = MTLDepthStencilDescriptor()
        d.isDepthWriteEnabled = true
        d.depthCompareFunction = .less
        depthState = device.makeDepthStencilState(descriptor: d)
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    // MARK: - Draw

    func draw(in view: MTKView) {
        guard
            let pass = view.currentRenderPassDescriptor,
            let drawable = view.currentDrawable,
            let cmd = queue.makeCommandBuffer(),
            let enc = cmd.makeRenderCommandEncoder(descriptor: pass)
        else { return }

//        let t = Float(CACurrentMediaTime() - startTime)

        let aspect = Float(view.drawableSize.width / max(view.drawableSize.height, 1))
        let proj = perspective(fovyRadians: 55 * (.pi / 180), aspect: aspect, near: 0.1, far: 50.0)

        let cameraPos = SIMD3<Float>(0, 0.1, 2.4)
        let viewM = lookAt(eye: cameraPos, center: SIMD3<Float>(0, 0, 0), up: SIMD3<Float>(0, 1, 0))

        let now = CACurrentMediaTime()
        let dt = Float(now - lastFrameTime)
        lastFrameTime = now

        // Inertia when not dragging
        if !isInteracting {
            let speed = simd_length(angularVelocity)
            if speed > 0.00001 && dt.isFinite {
                let axis = simd_normalize(angularVelocity)
                let angle = speed * dt
                orientation = simd_quatf(angle: angle, axis: axis) * orientation
                orientation = simd_normalize(orientation)

                // Damping (smooth stop)
                let damping: Float = 0.90
                angularVelocity *= pow(damping, dt * 60.0)
            }
        }

        // Camera (zoom)
        let cameraPos = SIMD3<Float>(0, 0.1, distance)
        let viewM = lookAt(eye: cameraPos, center: SIMD3<Float>(0, 0, 0), up: SIMD3<Float>(0, 1, 0))

        // Model from quaternion
        let model = rotationMatrix(from: orientation)




        let mvp = proj * viewM * model

        // normal matrix from model (upper-left 3x3 inverse-transpose)
        let upperLeft3x3 = simd_float3x3(columns: (
            SIMD3<Float>(model.columns.0.x, model.columns.0.y, model.columns.0.z),
            SIMD3<Float>(model.columns.1.x, model.columns.1.y, model.columns.1.z),
            SIMD3<Float>(model.columns.2.x, model.columns.2.y, model.columns.2.z)
        ))

        let normalMatrix = upperLeft3x3.inverse.transpose


        var uniforms = Uniforms(
            mvp: mvp,
            model: model,
            normalMatrix: normalMatrix,
            lightDir: simd_normalize(SIMD3<Float>(0.4, 0.9, 0.6)),
            cameraPos: cameraPos
        )

        enc.setRenderPipelineState(pipeline)
        enc.setDepthStencilState(depthState)

        enc.setVertexBuffer(vertexBuffer, offset: 0, index: 0)
        enc.setVertexBytes(&uniforms, length: MemoryLayout<Uniforms>.stride, index: 1)
        enc.setFragmentBytes(&uniforms, length: MemoryLayout<Uniforms>.stride, index: 1)

        // Prism vertices are already expanded into triangles
        let vertexCount = vertexBuffer.length / MemoryLayout<Vertex>.stride
        enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: vertexCount)

        enc.endEncoding()
        cmd.present(drawable)
        cmd.commit()
    }

    // MARK: - Geometry

    private func makePrismVertexBuffer(device: MTLDevice) -> MTLBuffer {
        // Base 2D triangle points (CCW)
        let top  = SIMD2<Float>( 0.0,  0.6)
        let left = SIMD2<Float>(-0.6, -0.6)
        let right = SIMD2<Float>( 0.6, -0.6)

        let depth: Float = 0.06
        let zF: Float =  depth * 0.5
        let zB: Float = -depth * 0.5

        func v(_ p: SIMD2<Float>, _ z: Float, _ n: SIMD3<Float>) -> Vertex {
            Vertex(position: SIMD3<Float>(p.x, p.y, z), normal: n)
        }

        let nFront = SIMD3<Float>(0, 0, 1)
        let nBack  = SIMD3<Float>(0, 0, -1)

        // Side normals (approx outward in XY)
        func sideNormal(a: SIMD2<Float>, b: SIMD2<Float>) -> SIMD3<Float> {
            let e = b - a
            let n2 = SIMD2<Float>(e.y, -e.x)
            let n = SIMD3<Float>(n2.x, n2.y, 0)
            return simd_normalize(n)
        }

        let nTL = sideNormal(a: top,  b: left)
        let nLR = sideNormal(a: left, b: right)
        let nRT = sideNormal(a: right,b: top)

        // Build triangles explicitly (no index buffer) for correct per-face normals
        var tris: [Vertex] = []

        // Front face (CCW)
        tris += [ v(top, zF, nFront), v(left, zF, nFront), v(right, zF, nFront) ]

        // Back face (reverse winding so it faces outward)
        tris += [ v(top, zB, nBack), v(right, zB, nBack), v(left, zB, nBack) ]

        // Side face helper: rectangle A->B extruded, 2 triangles
        func addSide(a: SIMD2<Float>, b: SIMD2<Float>, n: SIMD3<Float>) {
            let aF = v(a, zF, n)
            let bF = v(b, zF, n)
            let bB = v(b, zB, n)
            let aB = v(a, zB, n)

            // Ensure CCW from outside
            tris += [ aF, bF, bB ]
            tris += [ aF, bB, aB ]
        }

        addSide(a: top,  b: left,  n: nTL)
        addSide(a: left, b: right, n: nLR)
        addSide(a: right,b: top,   n: nRT)

        return device.makeBuffer(bytes: tris,
                                 length: MemoryLayout<Vertex>.stride * tris.count,
                                 options: [])!
    }

    // MARK: - Math

    private func perspective(fovyRadians: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let y: Float = 1 / tan(fovyRadians * 0.5)
        let x: Float = y / aspect
        let z: Float = far / (near - far)
        let w: Float = (near * far) / (near - far)

        return simd_float4x4(columns: (
            SIMD4<Float>(x, 0, 0, 0),
            SIMD4<Float>(0, y, 0, 0),
            SIMD4<Float>(0, 0, z, -1),
            SIMD4<Float>(0, 0, w, 0)
        ))
    }

    private func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let s = simd_normalize(simd_cross(f, up))
        let u = simd_cross(s, f)

        return simd_float4x4(columns: (
            SIMD4<Float>( s.x,  s.y,  s.z, 0),
            SIMD4<Float>( u.x,  u.y,  u.z, 0),
            SIMD4<Float>(-f.x, -f.y, -f.z, 0),
            SIMD4<Float>(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)
        ))
    }

    private func rotation(radians: Float, axis: SIMD3<Float>) -> simd_float4x4 {
        let a = simd_normalize(axis)
        let ct = cos(radians)
        let st = sin(radians)
        let ci = 1 - ct

        let m00 = ct + a.x*a.x*ci
        let m01 = a.x*a.y*ci - a.z*st
        let m02 = a.x*a.z*ci + a.y*st

        let m10 = a.y*a.x*ci + a.z*st
        let m11 = ct + a.y*a.y*ci
        let m12 = a.y*a.z*ci - a.x*st

        let m20 = a.z*a.x*ci - a.y*st
        let m21 = a.z*a.y*ci + a.x*st
        let m22 = ct + a.z*a.z*ci

        return simd_float4x4(columns: (
            SIMD4<Float>(m00, m10, m20, 0),
            SIMD4<Float>(m01, m11, m21, 0),
            SIMD4<Float>(m02, m12, m22, 0),
            SIMD4<Float>(0,   0,   0,   1)
        ))
    }

}
