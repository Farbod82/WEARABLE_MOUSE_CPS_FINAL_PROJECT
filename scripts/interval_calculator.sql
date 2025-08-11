SELECT
  s.ts,
  LAG(s.ts) OVER (ORDER BY s.ts) AS prev_ts,
  s.ts - LAG(s.ts) OVER (ORDER BY s.ts) AS ts_diff
FROM slices s
WHERE s.name = 'click'
ORDER BY s.ts;