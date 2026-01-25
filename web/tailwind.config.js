/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'ros-blue': '#22577a',
        'ros-green': '#38a3a5',
        'ros-light': '#57cc99',
        'ros-mint': '#80ed99',
        'ros-pale': '#c7f9cc',
      },
    },
  },
  plugins: [],
}
