// Frozen differential reference: a73ff9c59e44cb76f21343e2877133ceb875f9b5 (round 10).
void UltraHub::ledger_line_() {
  if (this->rw_uart_ == nullptr)
    return;
  const uint32_t now = millis();
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr && this->port_->core().line_on() != this->line_on_) {
    this->line_on_ = !this->line_on_;
    if (this->line_on_)
      this->keeper_.power_on(this->port_->core().line_ms());
    else
      this->keeper_.cut(this->port_->core().line_ms());
  }
#endif
  this->lit_rx_.assign(this->poller_.receivers(), 0);
  if (this->boards_powered_()) {
    for (auto *l : this->lights_) {
      if (l->get_board() < this->lit_rx_.size() && l->is_lit(now))
        this->lit_rx_[l->get_board()] = 1;
    }
  }
  for (uint16_t i = 0; i < this->lit_rx_.size(); i++)
    this->keeper_.lit_tick(i, this->lit_rx_[i] != 0, now);
}
