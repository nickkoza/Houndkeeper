# [ja6] — Names a DimOS module of ours must not use

**Status:** Decided · **Date:** 2026-10-04 · **Source:** the project service's first run against a relay (project-app
[eTc]), DimOS fork at a7492431f · **Serves:** every houndkeeper DimOS module: the phone cockpit's feeds ([Ndt], [F2q],
[vRJ]), the project service ([eTc]) · **ID:** [ja6]

## Context

DimOS's `Module` keeps its own state on the instance under private names, set in its constructor (at a7492431f, among
them `_loop` and `_loop_thread`, the module's event loop). A subclass method of the same name is hidden by that
instance attribute without a word: `threading.Thread(target=self._loop)` then gets the event loop, and the thread dies
at once with "'_UnixSelectorEventLoop' object is not callable", printed to the worker's stderr while the module
otherwise starts cleanly. Observed 4 Oct 2026: the project service's first run, and the Morning page's feed
(`MorningLogFeed`, [Ndt]), which had carried the same name since it was written and so never published from a live run;
its tests call `check()` directly and could not see it.

## Decisions

1. [gB6] **No houndkeeper module defines a name DimOS's `Module` sets on its instances.** Held by
   `tests/test_module_names.py`, which reads the names `Module` (and its bases) assign on `self` from DimOS's own source
   and fails on any houndkeeper module class that defines one, so the class of bug is caught on a DimOS update as well.
   The two feeds' thread methods are named `_watch`.
