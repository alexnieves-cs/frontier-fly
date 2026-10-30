// Wall-clock budgets hold only for uninstrumented runs: `npm run test:coverage` sets COVERAGE=1,
// and v8 instrumentation slows hot loops past them.
export const checkTiming = !process.env.COVERAGE;
