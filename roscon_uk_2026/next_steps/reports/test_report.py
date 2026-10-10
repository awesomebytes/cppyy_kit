"""Independent controlled fixtures. No MCAP or native dependencies required."""
import importlib
import os
import unittest
core = importlib.import_module(os.environ.get('REPORT_CORE_MODULE', 'roscon_uk_2026.next_steps.reports.report_core'))
check_frames, match_images = core.check_frames, core.match_images
rank_windows, trace_svg = core.rank_windows, core.trace_svg


class ReportChecks(unittest.TestCase):
    def test_known_five_windows(self):
        base = 1_750_000_000_000_000_007
        times = [base+i*10 for i in range(13)]
        scores = [0, 1, 0, 7, 0, 5, 0, 9, 0, 3, 0, 0, 100]
        windows = rank_windows(times, scores, 20)
        self.assertEqual([w.start_ns-base for w in windows], [60,20,40,80,0])
        self.assertEqual([w.score for w in windows], [4.5,3.5,2.5,1.5,.5])
        self.assertEqual([w.representative_ns-base for w in windows], [70,30,50,90,10])
        for left in windows:
            for right in windows:
                if left is not right:
                    self.assertFalse(left.start_ns < right.end_ns and right.start_ns < left.end_ns)

    def test_ties_and_half_open_boundary(self):
        windows = rank_windows([0,10,20,30,40,50,60], [0.1]*7, 20)
        self.assertEqual([w.start_ns for w in windows], [0,20,40])
        self.assertTrue(all(w.samples == 2 for w in windows))
        self.assertEqual(windows[0].representative_ns, 0)
        self.assertEqual(rank_windows([0], [1], 1), [])

    def test_nearest_precision_boundary_duplicates_missing(self):
        base = 1_750_000_000_000_000_007
        images = [base,base,base+10,base+20]
        query = [base+5,base+10,base+26,base+27]
        result = match_images(query, images, query_clock='header', image_clock='header', tolerance_ns=6)
        self.assertEqual([r['image_index'] for r in result], [0,2,3,None])
        self.assertEqual([r['delta_ns'] for r in result], [-5,0,-6,None])
        self.assertEqual(match_images([base],images,query_clock='x',image_clock='x',direction='backward')[0]['image_index'],1)
        self.assertEqual(match_images([base],images,query_clock='x',image_clock='x',direction='forward')[0]['image_index'],0)
        self.assertEqual(match_images([base],[],query_clock='x',image_clock='x')[0]['status'],'unmatched')

    def test_query_rank_order_and_validation(self):
        result = match_images([30,10], [10,20,30], query_clock='x', image_clock='x')
        self.assertEqual([r['image_index'] for r in result], [2,0])
        with self.assertRaises(TypeError):
            match_images([30.0,10.0],[10],query_clock='x',image_clock='x')
        with self.assertRaises(ValueError):
            match_images([10],[10],query_clock='header',image_clock='log')
        with self.assertRaises(ValueError):
            match_images([10],[20,10],query_clock='x',image_clock='x')
        for times, scores in [([1,1],[1,2]), ([2,1],[1,2]), ([1,2],[1,float('nan')])]:
            with self.assertRaises(ValueError):
                rank_windows(times,scores,1)
        with self.assertRaises(ValueError):
            check_frames('map','world','map')

    def test_plot_escapes_metadata_and_bounds(self):
        result = trace_svg(list(range(10_000)), [('<script>alert(1)</script>', [1]*10_000)])
        self.assertNotIn('<script>',result)
        self.assertLess(len(result),40_000)
        self.assertIn('&lt;script&gt;',result)


if __name__ == '__main__':
    unittest.main()
