import { useCameraImage } from '@/hooks/useCameraImage';

interface CameraViewProps {
  namespace: string;
}

export const CameraView = ({ namespace }: CameraViewProps) => {
  const { imageData, isReceiving, fps } = useCameraImage(namespace);

  return (
    <div className="bg-[rgb(17,24,39)] rounded-xl overflow-hidden h-full flex flex-col">
      {/* Header */}
      <div className="px-3 py-2 flex items-center justify-between border-b border-gray-700">
        <div className="flex items-center gap-2">
          <svg className="w-4 h-4 text-blue-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
          </svg>
          <span className="text-xs text-gray-300 font-medium">Camera</span>
        </div>
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${isReceiving ? 'bg-green-500' : 'bg-gray-500'}`} />
          {isReceiving && (
            <span className="text-xs text-gray-400">{fps} FPS</span>
          )}
        </div>
      </div>

      {/* Image Display */}
      <div className="flex-1 flex items-center justify-center bg-black min-h-[180px]">
        {imageData ? (
          <img
            src={imageData}
            alt="Robot Camera"
            className="max-w-full max-h-full object-contain"
          />
        ) : (
          <div className="text-gray-500 text-sm flex flex-col items-center gap-2">
            <svg className="w-8 h-8 text-gray-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
            </svg>
            <span>No image</span>
          </div>
        )}
      </div>
    </div>
  );
};
