import {createPlanningWorkerService} from './planning-worker-core.mjs?v=20261018-perf-4';
import {createPlanningWorkerEndpoint} from './planning-worker-protocol.mjs?v=20261018-perf-4';

const service=createPlanningWorkerService();
const endpoint=createPlanningWorkerEndpoint({service,postMessage:message=>self.postMessage(message)});
self.onmessage=event=>endpoint.handle(event.data);
