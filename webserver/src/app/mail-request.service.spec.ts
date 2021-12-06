import { TestBed } from '@angular/core/testing';

import { MailRequestService } from './mail-request.service';

describe('MailRequestService', () => {
  let service: MailRequestService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(MailRequestService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
