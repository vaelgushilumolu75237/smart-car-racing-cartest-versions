"""Current-frame dark evidence may resolve a leading block, never white road."""
import unittest
from unittest.mock import patch
from test_fast_gray_mask_equivalence import _BufferImage, _expanded_frame
import config, fast_gray, fast_topology


class RingDarkBlockTests(unittest.TestCase):
    def frame(self, top=55, other_dark=(50,50,50)):
        cells=bytearray([top]*4800)
        for block,(x0,y0,x1,y1) in enumerate(config.GRAY_THRESHOLD_BLOCKS):
            if not block:continue
            for y in range(y0,y1):
                for x in range(x0,x1):
                    cells[y*80+x]=other_dark[block-1] if x<40 else 220
        return _BufferImage(_expanded_frame(cells))

    def test_resolves_only_known_black_with_complete_topology(self):
        builder=fast_gray.FastGrayMasks(config)
        info=builder.build(self.frame())
        self.assertFalse(info['complete'])
        road_before=bytes(info['road_mask'])
        self.assertTrue(builder.resolve_confirmed_ring_dark_block(info))
        self.assertEqual(bytes(info['road_mask']),road_before)
        self.assertTrue(info['complete'])
        self.assertTrue(all(info['known_region'][8*80:20*80]))
        self.assertTrue(all(info['edge_mask'][8*80:20*80]))
        self.assertTrue(fast_topology.FastTopologyGuard(config).check(info)['complete'])

    def test_bright_ambiguous_or_disagreeing_current_references_remain_unknown(self):
        for image in (self.frame(220),self.frame(100),
                      self.frame(other_dark=(50,100,50)),
                      self.frame(other_dark=(50,220,50))):
            b=fast_gray.FastGrayMasks(config);info=b.build(image)
            self.assertFalse(b.resolve_confirmed_ring_dark_block(info))
            self.assertFalse(info['complete'])

    def test_no_second_unknown_block_or_foreign_mask_can_be_resolved(self):
        b=fast_gray.FastGrayMasks(config);info=b.build(self.frame())
        self.assertFalse(b.resolve_confirmed_ring_dark_block(dict(info)))
        b.trusted[2]=False
        self.assertFalse(b.resolve_confirmed_ring_dark_block(info))

    def test_next_frame_clears_resolution_and_does_not_reuse_dark_reference(self):
        b=fast_gray.FastGrayMasks(config);info=b.build(self.frame())
        self.assertTrue(b.resolve_confirmed_ring_dark_block(info))
        info=b.build(self.frame(220))
        self.assertFalse(info['ring_dark_block_resolved'])
        self.assertFalse(b.resolve_confirmed_ring_dark_block(info))
        self.assertFalse(any(info['known_region'][8*80:20*80]))

    def sliver_frame(self, populations):
        cells=bytearray([55]*4800)
        for block,(x0,y0,x1,y1) in enumerate(config.GRAY_THRESHOLD_BLOCKS):
            if not block:continue
            n=populations[block-1]
            for j in range((x1-x0)*(y1-y0)):
                y=y0+j//(x1-x0);x=x0+j%(x1-x0)
                cells[y*80+x]=220 if j<n else 50
        return _BufferImage(_expanded_frame(cells))

    def test_crossing_five_percent_in_one_peer_keeps_two_strong_references(self):
        for n in (49,48,47,46,1):
            b=fast_gray.FastGrayMasks(config)
            info=b.build(self.sliver_frame((n,400,600)))
            before=bytes(info['road_mask'])
            self.assertTrue(b.resolve_confirmed_ring_dark_block(info),n)
            self.assertEqual(bytes(info['road_mask']),before)
            self.assertTrue(fast_topology.FastTopologyGuard(config).check(info)['complete'])

    def test_one_populated_reference_is_insufficient(self):
        b=fast_gray.FastGrayMasks(config)
        info=b.build(self.sliver_frame((46,1,600)))
        self.assertTrue(all(b.trusted[1:]))
        self.assertFalse(b.resolve_confirmed_ring_dark_block(info))
        self.assertFalse(info['complete'])

    def test_unpopulated_peer_dark_disagreement_still_rejects(self):
        b=fast_gray.FastGrayMasks(config)
        info=b.build(self.sliver_frame((46,400,600)))
        b.class_means[1]=(100.,220.)
        self.assertFalse(b.resolve_confirmed_ring_dark_block(info))

    def test_unpopulated_but_weak_contrast_peer_still_rejects(self):
        b=fast_gray.FastGrayMasks(config)
        info=b.build(self.sliver_frame((46,400,600)))
        b.contrasts[1]=b.min_contrast
        self.assertFalse(b.resolve_confirmed_ring_dark_block(info))

    def test_ring_resolver_is_opt_in_and_does_not_change_ordinary_mask(self):
        b=fast_gray.FastGrayMasks(config)
        info=b.build(self.sliver_frame((46,400,600)))
        self.assertFalse(info['complete'])
        self.assertFalse(any(info['known_region'][8*80:20*80]))
        self.assertFalse(info['ring_dark_block_resolved'])

    def test_mixed_gray_runtime_is_rejected(self):
        import test_fast_main_ring_slope
        import main
        with patch.object(fast_gray,'RING_DARK_BLOCK_API_VERSION',0):
            with self.assertRaisesRegex(RuntimeError,'dark-block API'):
                main._load_track_elements_runtime()


if __name__=='__main__':unittest.main()
