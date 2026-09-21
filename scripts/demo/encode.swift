import Foundation
import AVFoundation
import AppKit

// Encode Pillow-rendered frames using macOS's native H.264 encoder.
let args = CommandLine.arguments
guard args.count == 3 else { fatalError("Usage: encode FRAMES_DIR OUTPUT.mp4") }
let output = URL(fileURLWithPath: args[2])
if FileManager.default.fileExists(atPath: output.path) { try FileManager.default.removeItem(at: output) }
let writer = try AVAssetWriter(outputURL: output, fileType: .mp4)
let input = AVAssetWriterInput(mediaType: .video, outputSettings: [
    AVVideoCodecKey: AVVideoCodecType.h264,
    AVVideoWidthKey: 1280, AVVideoHeightKey: 720,
    AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: 4_000_000,
                                    AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel,
                                    AVVideoMaxKeyFrameIntervalKey: 48]
])
input.expectsMediaDataInRealTime = false
let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input, sourcePixelBufferAttributes: [
    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32ARGB,
    kCVPixelBufferWidthKey as String: 1280, kCVPixelBufferHeightKey as String: 720,
    kCVPixelBufferCGImageCompatibilityKey as String: true,
    kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
])
writer.add(input)
writer.shouldOptimizeForNetworkUse = true
guard writer.startWriting() else { fatalError("Start: \(String(describing: writer.error))") }
writer.startSession(atSourceTime: .zero)
for frame in 0..<1224 {
    while !input.isReadyForMoreMediaData {
        if writer.status == .failed { fatalError("Encoder failed: \(String(describing: writer.error))") }
        Thread.sleep(forTimeInterval: 0.005)
    }
    try autoreleasepool {
        let url = URL(fileURLWithPath: args[1]).appendingPathComponent(String(format:"%05d.jpg",frame))
        guard let source=CGImageSourceCreateWithURL(url as CFURL,nil),let image=CGImageSourceCreateImageAtIndex(source,0,nil) else { fatalError("Missing frame \(frame)") }
        var pixel: CVPixelBuffer?
        guard CVPixelBufferPoolCreatePixelBuffer(nil,adaptor.pixelBufferPool!,&pixel)==kCVReturnSuccess,let pixel=pixel else { fatalError("Pixel buffer") }
        CVPixelBufferLockBaseAddress(pixel,[])
        let ctx=CGContext(data:CVPixelBufferGetBaseAddress(pixel),width:1280,height:720,bitsPerComponent:8,bytesPerRow:CVPixelBufferGetBytesPerRow(pixel),space:CGColorSpaceCreateDeviceRGB(),bitmapInfo:CGImageAlphaInfo.noneSkipFirst.rawValue)!
        ctx.draw(image,in:CGRect(x:0,y:0,width:1280,height:720))
        CVPixelBufferUnlockBaseAddress(pixel,[])
        guard adaptor.append(pixel,withPresentationTime:CMTime(value:Int64(frame),timescale:24)) else { fatalError("Append: \(String(describing: writer.error))") }
    }
    if frame % 240 == 0 { print("Encoded \(frame)/1224") }
}
input.markAsFinished()
let semaphore=DispatchSemaphore(value:0)
writer.finishWriting { semaphore.signal() }
semaphore.wait()
guard writer.status == .completed else { fatalError("Finish: \(String(describing: writer.error))") }
print("Created \(output.path)")
let asset=AVURLAsset(url:output)
let duration=asset.duration.seconds
guard abs(duration-51)<0.1 else { fatalError("Unexpected duration: \(duration)") }
let generator=AVAssetImageGenerator(asset:asset)
generator.appliesPreferredTrackTransform=true
for second in [2,20,28,43,48] {
    let cg=try generator.copyCGImage(at:CMTime(seconds:Double(second),preferredTimescale:24),actualTime:nil)
    let rep=NSBitmapImageRep(cgImage:cg)
    let image=rep.representation(using:.png,properties:[:])!
    try image.write(to:URL(fileURLWithPath:args[1]).deletingLastPathComponent().appendingPathComponent("decoded-\(second).png"))
}
print("Verified duration: \(duration)s; extracted five encoded frames for visual review")
