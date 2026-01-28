export default {
  input: 'src/main.js',
  output: {
    dir: 'output/',
    format: 'es',
    entryFileNames: '[name].js'
  },
  external: ['cs_script/point_script']
}