
<template>
  <div class="info">
    <canvas ref="lidarCanvas" width="600" height="600"></canvas>
    <p>X: {{ state[0] }}, Y: {{ state[1] }}<b> {{ ((Math.PI / 180) * state[2]).toFixed(1) }}</b></p>
    <p>Best match distance: {{ best[1] }}  angle: {{ best[2] }}</p>
  </div>
</template>

<style>
.info {
  border-radius: 5rem;
  border-color: black;
  margin: 3rem;
}
</style>

<script>
export default {
  data() {
    return { scanPoints: [], 
             ws: null,
             state: [],
            best: [] }
  },
  mounted() {
  this.ctx = this.$refs.lidarCanvas.getContext('2d')

  this.ws = new WebSocket('ws://172.20.10.7:3000')
  // when port recieves an event, parse data
  this.ws.onmessage = (event) => {
    const data = JSON.parse(event.data)
    this.scanPoints = data.scan
    this.state = data.state
    this.best = data.best_match
    }
  },
  watch: {
    scanPoints() {
      this.draw()
    }
  },
  methods: {
    draw() {
      // clear canvas
      const canvas = this.$refs.lidarCanvas
      this.ctx.clearRect(0, 0, canvas.width, canvas.height)

      // plot origin and landmark start for robot in red and blue
      this.ctx.fillStyle = 'red'
      this.ctx.fillRect(300, 300, 1, 1)
      this.ctx.fillStyle = 'blue'
      this.ctx.fillRect(600, 300, 1, 1)

      // convert polar to cartesian
      this.ctx.fillStyle = 'lime'
      for (const meas of this.scanPoints) {
        const [quality, angle, distance] = meas
        const angle_rad = angle *  (180 / Math.PI)
        const x_mm = distance * Math.cos(angle_rad)
        const y_mm = distance * Math.sin(angle_rad)

        // translate onto canvas grid of 600 x 600 and plot
        const x_px = (x_mm + 3000) / 10
        const y_px = (y_mm + 3000) / 10
        this.ctx.fillRect(x_px, y_px, 2, 2)
      }

      const [x, y, heading] = this.state

      // from the origin, measure the change to map relative point we are at (mm)
      const dx = 0 - (x * 1000)
      const dy = 0 - (y * 1000) 

      // use change times heading difference to calculate coordinates
      const local_x = ((dx * Math.cos(heading) + dy * Math.sin(heading)) + 3000) / 10
      const local_y = (-dx * Math.sin(heading) + dy * Math.cos(heading) + 3000) / 10

      this.ctx.fillStyle = 'black'
      this.ctx.fillRect(local_x, local_y, 3, 3)

      const [qual, dist, ang] = this.best
      const ang_rad = ang *  (180 / Math.PI)

      const translate_x = Math.cos(ang_rad) * dist
      const translate_y = Math.sin(ang_rad) * dist
      this.ctx.fillStyle = 'red'
      this.ctx.fillRect(((translate_x + 3000)/10), ((translate_y + 3000) /10), 2, 2)
    }
  }
}
</script>

