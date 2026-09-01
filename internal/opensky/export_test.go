package opensky

import (
	"errors"
	"net/http"
	"net/http/httptest"
	"net/url"
)

// newTestClient points a client at a stand-in server by rewriting the host of
// every outbound request, which keeps the real endpoint constants in the
// package rather than turning them into fields nobody sets in production.
func newTestClient(srv *httptest.Server, id, secret string) *Client {
	base, _ := url.Parse(srv.URL)
	c := New(id, secret)
	c.HTTP = &http.Client{Transport: rewriteHost{base: base, next: srv.Client().Transport}}
	return c
}

type rewriteHost struct {
	base *url.URL
	next http.RoundTripper
}

func (r rewriteHost) RoundTrip(req *http.Request) (*http.Response, error) {
	req = req.Clone(req.Context())
	req.URL.Scheme = r.base.Scheme
	req.URL.Host = r.base.Host
	next := r.next
	if next == nil {
		next = http.DefaultTransport
	}
	return next.RoundTrip(req)
}

func asRateLimit(err error, target **RateLimitError) bool {
	return errors.As(err, target)
}
