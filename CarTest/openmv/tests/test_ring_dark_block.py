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

    def test_mixed_gray_runtime_is_rejected(self):
        import test_fast_main_ring_slope
        import main
        with patch.object(fast_gray,'RING_DARK_BLOCK_API_VERSION',0):
            with self.assertRaisesRegex(RuntimeError,'dark-block API'):
                main._load_track_elements_runtime()


if __name__=='__main__':unittest.main()
