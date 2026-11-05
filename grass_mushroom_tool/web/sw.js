/* 最小 Service Worker：只用来让浏览器允许「添加到主屏幕」。
   采集数据必须实时拉取，因此不做离线缓存，全部走网络。 */
self.addEventListener("install", () => self.skipWaiting());
self.addEventListener("activate", (event) => event.waitUntil(self.clients.claim()));
self.addEventListener("fetch", () => {
  /* 不调用 respondWith，交给浏览器正常走网络 */
});
