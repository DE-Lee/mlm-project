// roslib 타입 정의
declare module 'roslib' {
  export class Ros {
    constructor(options?: { url?: string });
    on(event: 'connection' | 'error' | 'close', callback: (error?: Error) => void): void;
    close(): void;
    isConnected: boolean;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      throttle_rate?: number;
      queue_size?: number;
      latch?: boolean;
    });
    subscribe(callback: (message: unknown) => void): void;
    unsubscribe(): void;
    publish(message: Message): void;
  }

  export class Message {
    constructor(values: Record<string, unknown>);
  }

  export class Service {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(request: ServiceRequest, callback: (response: unknown) => void, errorCallback?: (error: Error) => void): void;
  }

  export class ServiceRequest {
    constructor(values: Record<string, unknown>);
  }

  export class Param {
    constructor(options: {
      ros: Ros;
      name: string;
    });
    get(callback: (value: unknown) => void): void;
    set(value: unknown, callback?: () => void): void;
  }

  export class ActionClient {
    constructor(options: {
      ros: Ros;
      serverName: string;
      actionName: string;
    });
    dispose(): void;
  }

  export class Goal {
    constructor(options: {
      actionClient: ActionClient;
      goalMessage: unknown;
    });
    on(event: string, callback: (result: unknown) => void): void;
    send(): void;
    cancel(): void;
  }
}
