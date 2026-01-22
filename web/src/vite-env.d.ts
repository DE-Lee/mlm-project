/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_ROS_BRIDGE_URL: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
