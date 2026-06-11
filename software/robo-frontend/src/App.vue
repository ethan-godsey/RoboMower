
<template>
  <canvas ref="lidarCanvas" width="600" height="600"></canvas>
</template>

<script>
export default {
  data() {
    return { scanPoints: [], ws: null }
  },
  mounted() {
  this.ctx = this.$refs.lidarCanvas.getContext('2d')

  this.ws = new WebSocket('ws://172.20.10.7:3000')
  this.ws.onmessage = (event) => {
    this.scanPoints = JSON.parse(event.data)
    }
  },
  watch: {
    scanPoints() {
      this.draw()
    }
  },
  methods: {
    draw() {
      // clear canvas, convert polar -> cartesian, plot points
      const canvas = this.$refs.lidarCanvas
      this.ctx.clearRect(0, 0, canvas.width, canvas.height)
      this.ctx.fillStyle = 'lime'
      for (const meas of this.scanPoints) {
        const [quality, angle, distance] = meas
        const angle_rad = angle * Math.PI / 180
        const x_mm = distance * Math.cos(angle_rad)
        const y_mm = distance * Math.sin(angle_rad)
        const x_px = (x_mm + 6000) / 20
        const y_px = (y_mm + 6000) / 20
        this.ctx.fillRect(x_px, y_px, 2, 2)
      }
    }
  }
}
</script>

